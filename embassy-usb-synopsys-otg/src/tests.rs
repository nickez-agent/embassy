use core::pin::{Pin, pin};
use core::sync::atomic::AtomicUsize;
use core::task::{Context, Waker};
use std::sync::Arc;
use std::task::Wake;

use embassy_usb_driver::{ControlPipe as _, Driver as _};

use super::*;

// RAM does not pop RX status entries or clear RXFLVL as real MMIO does. The
// receive interrupt wakes after consuming our single queued packet; use that
// wake to model the FIFO becoming empty, and record it for cancellation tests.
struct RxWaker {
    regs: Otg,
    wakes: AtomicUsize,
}

impl Wake for RxWaker {
    fn wake(self: Arc<Self>) {
        self.wake_by_ref();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.regs.gintsts().modify(|w| w.set_rxflvl(false));
        self.wakes.fetch_add(1, Ordering::Relaxed);
    }
}

struct Hardware<'a> {
    regs: Otg,
    state: State<'a>,
    wake: Arc<RxWaker>,
    waker: Waker,
}

impl Hardware<'_> {
    fn poll<F: Future>(&self, future: Pin<&mut F>) -> Poll<F::Output> {
        future.poll(&mut Context::from_waker(&self.waker))
    }

    fn poll_once<F: Future>(&self, future: F) -> Poll<F::Output> {
        self.poll(pin!(future))
    }

    fn receive(&self, packet: vals::Pktstsd, len: u16, word: u32) {
        let mut status = regs::Grxsts::default();
        status.set_epnum(0);
        status.set_bcnt(len);
        status.set_pktstsd(packet);
        // Populate the register that the hardware normally writes.
        unsafe { self.regs.grxstsp().as_ptr().write(status) };
        self.regs.fifo(0).write_value(regs::Fifo(word));
        self.regs.gintsts().write(|w| w.set_rxflvl(true));
        unsafe { on_interrupt(self.regs, &self.state) };
        assert!(!self.regs.gintsts().read().rxflvl());
    }

    fn setup_received(&self, byte: u8) {
        // No TX FIFO data remains in this model. FIFO flush completion would
        // require hardware to clear GRSTCTL.TXFFLSH, which RAM cannot do.
        self.regs.dieptsiz(0).write(|w| w.set_pktcnt(0));
        self.receive(vals::Pktstsd::SETUP_DATA_RX, 8, u32::from_ne_bytes([byte; 4]));
    }

    fn setup_completed(&self) {
        self.regs.gintsts().write(|w| w.set_oepint(true));
        unsafe { self.regs.daint().as_ptr().write(regs::Daint(1 << 16)) };
        self.regs.doepint(0).write(|w| w.set_stup(true));
        unsafe { on_interrupt(self.regs, &self.state) };
        self.regs.gintsts().write_value(regs::Gintsts::default());
    }

    fn setup(&self, byte: u8) {
        self.setup_received(byte);
        self.setup_completed();
    }

    fn out(&self, len: u16, byte: u8) {
        // Model a controller clearing EPENA when it finishes the OUT transfer.
        self.regs.doepctl(0).modify(|w| w.set_epena(false));
        self.receive(vals::Pktstsd::OUT_DATA_RX, len, u32::from_ne_bytes([byte; 4]));
    }
}

fn with_control(f: impl FnOnce(&Hardware<'_>, &mut ControlPipe<'_>)) {
    let mmio = UnsafeCell::new([0u32; 0x11000 / 4]);
    let regs = unsafe { Otg::from_ptr(mmio.get().cast()) };
    let storage = StateStorage::<9>::new(CriticalSectionRawMutex::new());
    let state = storage.as_state();
    let mut buffer = [0u8; 64];
    let instance = OtgInstance {
        regs,
        state,
        fifo_depth_words: 1024,
        tx_fifo_count: 9,
        extra_rx_fifo_words: 30,
        phy_type: PhyType::InternalHighSpeed,
        calculate_trdt_fn: |_| 6,
    };
    let (_bus, mut control) = Driver::new(&mut buffer, instance, Config::default()).start(64);
    // Hardware initialization uses self-clearing MMIO bits; initialize the
    // endpoint enable mirrors and available FIFO space directly for this model.
    state.ep_states[0].in_enabled.store(true, Ordering::Release);
    state.ep_states[0].out_enabled.store(true, Ordering::Release);
    unsafe { regs.dtxfsts(0).as_ptr().write(regs::Dtxfsts(64)) };
    let wake = Arc::new(RxWaker {
        regs,
        wakes: AtomicUsize::new(0),
    });
    let waker = Waker::from(wake.clone());
    state.cp_state.waker.register(&waker);
    state.ep_states[0].out_waker.register(&waker);
    let hw = Hardware {
        regs,
        state,
        wake,
        waker,
    };
    hw.setup(0x11);
    assert_eq!(hw.poll_once(control.setup()), Poll::Ready([0x11; 8]));
    f(&hw, &mut control);
}

fn overflow(hw: &Hardware<'_>, control: &mut ControlPipe<'_>) {
    let mut short = [0; 1];
    let mut read = pin!(control.data_out(&mut short, true, true));
    assert!(hw.poll(read.as_mut()).is_pending());
    hw.out(64, 0xa5);
    assert_eq!(hw.poll(read.as_mut()), Poll::Ready(Err(EndpointError::BufferOverflow)));
}

#[test]
fn test_setup_recovers_from_out_overflow() {
    with_control(|hw, control| {
        overflow(hw, control);
        hw.setup(0x22);
        assert_eq!(hw.poll_once(control.setup()), Poll::Ready([0x22; 8]));
        assert!(hw.regs.doepctl(0).read().epena());
        assert_eq!(hw.regs.doeptsiz(0).read().pktcnt(), 1);
        assert_eq!(hw.regs.doeptsiz(0).read().xfrsiz(), 64);

        let mut data = [0; 64];
        {
            let mut read = pin!(control.data_out(&mut data, true, true));
            assert!(hw.poll(read.as_mut()).is_pending());
            hw.out(64, 0x5a);
            assert_eq!(hw.poll(read.as_mut()), Poll::Ready(Ok(64)));
        }
        assert_eq!(data, [0x5a; 64]);
    });
}

#[test]
fn test_setup_recovers_status_after_out_overflow() {
    with_control(|hw, control| {
        overflow(hw, control);
        hw.setup(0x22);
        assert_eq!(hw.poll_once(control.setup()), Poll::Ready([0x22; 8]));
        let mut transfer = pin!(control.data_in(&[1], true, true));
        assert!(hw.poll(transfer.as_mut()).is_pending());
        hw.out(0, 0);
        assert_eq!(hw.poll(transfer.as_mut()), Poll::Ready(Ok(())));
    });
}

#[test]
fn test_setup_preserves_new_buffered_data() {
    with_control(|hw, control| {
        overflow(hw, control);
        hw.setup(0x22);
        hw.out(8, 0x5a);
        assert_eq!(hw.poll_once(control.setup()), Poll::Ready([0x22; 8]));
        // Do not rearm over the packet that still occupies the receive buffer.
        assert!(!hw.regs.doepctl(0).read().epena());
        let mut data = [0; 64];
        assert_eq!(
            hw.poll_once(control.data_out(&mut data, true, true)),
            Poll::Ready(Ok(8))
        );
        assert_eq!(&data[..8], &[0x5a; 8]);
        assert_eq!(&data[8..], &[0; 56]);
    });
}

#[test]
fn test_data_out_cancels_before_setup_completion() {
    with_control(|hw, control| {
        let mut data = [0; 64];
        {
            let mut transfer = pin!(control.data_out(&mut data, true, true));
            assert!(hw.poll(transfer.as_mut()).is_pending());
            let wakes = hw.wake.wakes.load(Ordering::Relaxed);
            hw.setup_received(0x22);
            assert!(hw.wake.wakes.load(Ordering::Relaxed) > wakes);
            assert_eq!(hw.poll(transfer.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
        }
        assert!(hw.poll_once(control.setup()).is_pending());
        hw.setup_completed();
        assert_eq!(hw.poll_once(control.setup()), Poll::Ready([0x22; 8]));
        assert_eq!(data, [0; 64]);
    });
}

#[test]
fn test_cancelled_data_out_preserves_new_packet() {
    with_control(|hw, control| {
        let mut data = [0; 64];
        {
            let mut transfer = pin!(control.data_out(&mut data, true, true));
            assert!(hw.poll(transfer.as_mut()).is_pending());
            hw.setup(0x22);
            hw.out(4, 0x5a);
            assert_eq!(hw.poll(transfer.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
        }
        assert_eq!(data, [0; 64]);
        assert_eq!(hw.poll_once(control.setup()), Poll::Ready([0x22; 8]));
        assert_eq!(
            hw.poll_once(control.data_out(&mut data, true, true)),
            Poll::Ready(Ok(4))
        );
        assert_eq!(&data[..4], &[0x5a; 4]);
    });
}

#[test]
fn test_data_in_cancels_at_each_wait() {
    for wait in 0..3 {
        with_control(|hw, control| {
            match wait {
                // Wait for the previous IN transfer, FIFO space, or OUT status.
                0 => hw.regs.diepctl(0).modify(|w| w.set_epena(true)),
                1 => unsafe { hw.regs.dtxfsts(0).as_ptr().write(regs::Dtxfsts(0)) },
                _ => {}
            }
            let mut transfer = pin!(control.data_in(&[1], true, true));
            assert!(hw.poll(transfer.as_mut()).is_pending());
            let wakes = hw.wake.wakes.load(Ordering::Relaxed);
            hw.setup_received(0x22);
            assert!(hw.wake.wakes.load(Ordering::Relaxed) > wakes);
            assert_eq!(hw.poll(transfer.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
        });
    }
}

#[test]
fn test_accept_cancels_on_setup() {
    with_control(|hw, control| {
        hw.regs.diepctl(0).modify(|w| w.set_epena(true));
        let mut accept = pin!(control.accept());
        assert!(hw.poll(accept.as_mut()).is_pending());
        hw.setup_received(0x22);
        assert_eq!(hw.poll(accept.as_mut()), Poll::Ready(()));
        assert_eq!(hw.regs.dieptsiz(0).read().pktcnt(), 0);
    });
}

#[test]
fn test_pending_setup_prevents_reject_and_set_address() {
    with_control(|hw, control| {
        hw.setup_received(0x22);
        assert_eq!(hw.poll_once(control.reject()), Poll::Ready(()));
        assert!(!hw.regs.diepctl(0).read().stall());
        assert!(!hw.regs.doepctl(0).read().stall());
        assert_eq!(hw.poll_once(control.accept_set_address(7)), Poll::Ready(()));
        assert_eq!(hw.regs.dcfg().read().dad(), 0);
        assert_eq!(hw.regs.dieptsiz(0).read().pktcnt(), 0);

        hw.setup_completed();
        assert_eq!(hw.poll_once(control.setup()), Poll::Ready([0x22; 8]));
        assert_eq!(hw.poll_once(control.accept_set_address(7)), Poll::Ready(()));
        assert_eq!(hw.regs.dcfg().read().dad(), 7);
        assert_eq!(hw.regs.dieptsiz(0).read().pktcnt(), 1);
        assert_eq!(hw.poll_once(control.reject()), Poll::Ready(()));
        assert!(hw.regs.diepctl(0).read().stall());
        assert!(hw.regs.doepctl(0).read().stall());
    });
}

#[test]
fn test_setup_keeps_only_latest_transfer() {
    with_control(|hw, control| {
        hw.setup(0x22);
        hw.out(4, 0xa5);
        hw.setup_received(0x33);
        assert!(hw.poll_once(control.setup()).is_pending());
        hw.setup_completed();
        hw.out(8, 0x5a);
        assert_eq!(hw.poll_once(control.setup()), Poll::Ready([0x33; 8]));
        let mut data = [0; 8];
        assert_eq!(
            hw.poll_once(control.data_out(&mut data, true, true)),
            Poll::Ready(Ok(8))
        );
        assert_eq!(data, [0x5a; 8]);
    });
}
