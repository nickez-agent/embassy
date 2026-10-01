use core::future::Future;
use core::pin::pin;
use core::task::Context;
use std::sync::Arc;
use std::task::{Wake, Waker};

use embassy_usb_driver::{Bus as _, Driver as _};

use super::*;

struct ClearRx(Otg);

impl Wake for ClearRx {
    fn wake(self: Arc<Self>) {
        // Model the RX level clearing after the queued packet has been drained.
        self.0.gintsts().modify(|w| w.set_rxflvl(false));
    }
}

fn poll_once<T>(future: impl Future<Output = T>, waker: &Waker) -> Poll<T> {
    pin!(future).as_mut().poll(&mut Context::from_waker(waker))
}

fn with_endpoints(test: impl FnOnce(Otg, State<'_>, Bus<'_>, Endpoint<'_, Out>, Endpoint<'_, Out>)) {
    let mmio = UnsafeCell::new([0u32; 0x11000 / 4]);
    // The aligned, initialized register storage outlives every handle used by the test.
    let regs = unsafe { Otg::from_ptr(mmio.get().cast()) };
    let storage = StateStorage::<3>::new(CriticalSectionRawMutex::new());
    let state = storage.as_state();
    let mut buffer = [0; 192];
    let instance = OtgInstance {
        regs,
        state,
        fifo_depth_words: 1024,
        tx_fifo_count: 3,
        extra_rx_fifo_words: 30,
        phy_type: PhyType::InternalHighSpeed,
        calculate_trdt_fn: |_| 6,
    };
    let mut driver = Driver::new(&mut buffer, instance, Config::default());
    let ep1 = driver.alloc_endpoint_out(EndpointType::Bulk, None, 64, 0).unwrap();
    let ep2 = driver.alloc_endpoint_out(EndpointType::Bulk, None, 64, 0).unwrap();
    let (mut bus, _control) = driver.start(64);
    bus.endpoint_set_enabled(ep1.info.addr, true);
    bus.endpoint_set_enabled(ep2.info.addr, true);
    test(regs, state, bus, ep1, ep2);
}

fn queue_packet(regs: Otg, index: usize, byte: u8) {
    // Model a completed one-packet transfer whose status/data still await FIFO service.
    regs.doepctl(index).modify(|w| w.set_epena(false));
    unsafe {
        regs.grxstsp()
            .as_ptr()
            .write(regs::Grxsts(index as u32 | (64 << 4) | (2 << 17)));
    }
    regs.fifo(0).write_value(regs::Fifo(u32::from_ne_bytes([byte; 4])));
    regs.gintsts().modify(|w| w.set_rxflvl(true));
}

#[test]
fn test_out_reconfiguration_discards_old_packet() {
    // A repeated enable is used by SET_INTERFACE and SET_CONFIGURATION as well as an
    // explicit disable/enable pair. Data can be buffered in RAM or still in the RX FIFO.
    for disable_first in [false, true] {
        for service_irq in [false, true] {
            with_endpoints(|regs, state, mut bus, mut ep1, mut ep2| {
                let waker = Waker::from(Arc::new(ClearRx(regs)));
                let mut output = [0; 64];
                assert!(poll_once(ep1.read(&mut output), &waker).is_pending());
                assert!(poll_once(ep2.read(&mut output), &waker).is_pending());

                queue_packet(regs, 2, 0x22);
                unsafe { on_interrupt(regs, &state) };
                queue_packet(regs, 1, 0xa5);
                if service_irq {
                    unsafe { on_interrupt(regs, &state) };
                }

                if disable_first {
                    bus.endpoint_set_enabled(ep1.info.addr, false);
                    assert_eq!(
                        poll_once(ep1.read(&mut output), &waker),
                        Poll::Ready(Err(EndpointError::Disabled))
                    );
                }
                bus.endpoint_set_enabled(ep1.info.addr, true);
                assert!(poll_once(ep1.read(&mut output), &waker).is_pending());
                assert_eq!(output, [0; 64]);
                assert!(regs.doepctl(1).read().epena());
                assert_eq!(regs.doeptsiz(1).read().pktcnt(), 1);

                assert_eq!(poll_once(ep2.read(&mut output), &waker), Poll::Ready(Ok(64)));
                assert_eq!(output, [0x22; 64]);

                queue_packet(regs, 1, 0x5a);
                unsafe { on_interrupt(regs, &state) };
                assert_eq!(poll_once(ep1.read(&mut output), &waker), Poll::Ready(Ok(64)));
                assert_eq!(output, [0x5a; 64]);
            });
        }
    }
}

#[test]
fn test_out_reconfiguration_preserves_other_endpoint_fifo_packet() {
    with_endpoints(|regs, _state, mut bus, mut ep1, mut ep2| {
        let waker = Waker::from(Arc::new(ClearRx(regs)));
        let mut output = [0; 64];
        assert!(poll_once(ep1.read(&mut output), &waker).is_pending());
        assert!(poll_once(ep2.read(&mut output), &waker).is_pending());

        // The target is idle, while the shared RX FIFO contains another endpoint's packet.
        regs.doepctl(1).modify(|w| w.set_epena(false));
        queue_packet(regs, 2, 0x22);
        bus.endpoint_set_enabled(ep1.info.addr, false);
        bus.endpoint_set_enabled(ep1.info.addr, true);

        assert!(poll_once(ep1.read(&mut output), &waker).is_pending());
        assert_eq!(poll_once(ep2.read(&mut output), &waker), Poll::Ready(Ok(64)));
        assert_eq!(output, [0x22; 64]);
        assert!(!regs.grstctl().read().rxfflsh());
    });
}

#[test]
fn test_out_reconfiguration_keeps_buffer_until_endpoint_stops() {
    with_endpoints(|regs, state, mut bus, mut ep1, _ep2| {
        let waker = Waker::from(Arc::new(ClearRx(regs)));
        let mut output = [0; 64];
        assert!(poll_once(ep1.read(&mut output), &waker).is_pending());
        queue_packet(regs, 1, 0xa5);
        unsafe { on_interrupt(regs, &state) };

        // Model a stop handshake whose interrupt flag is set but EPENA remains asserted.
        // The endpoint must stay unavailable until the hardware has actually stopped.
        regs.doepctl(1).modify(|w| w.set_epena(true));
        regs.gintsts().modify(|w| w.set_goutnakeff(true));
        regs.doepint(1).write(|w| w.set_epdisd(true));
        regs.doeptsiz(1).modify(|w| w.set_pktcnt(0));
        bus.endpoint_set_enabled(ep1.info.addr, true);

        assert_eq!(
            poll_once(ep1.read(&mut output), &waker),
            Poll::Ready(Err(EndpointError::Disabled))
        );
        assert_eq!(state.ep_states[1].out_size.load(Ordering::Acquire), 64);
        assert_eq!(regs.doeptsiz(1).read().pktcnt(), 0);
        assert_eq!(output, [0; 64]);
    });
}
