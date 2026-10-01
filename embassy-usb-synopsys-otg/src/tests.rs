use core::cell::RefCell;
use core::marker::PhantomData;
use core::task::{Context, Waker};
use std::sync::{Mutex, MutexGuard};

use embassy_usb_driver::{Bus as _, ControlPipe as _, Driver as _, EndpointIn as _};

use super::*;

// Keep tests that advance the mock clock separate from tests of a frozen clock.
static TEST_LOCK: Mutex<()> = Mutex::new(());

std::thread_local! {
    static HANDSHAKE_HOOK: RefCell<Option<Box<dyn FnMut()>>> = RefCell::new(None);
    static UNLOCK_HOOK: RefCell<Option<Box<dyn FnMut()>>> = RefCell::new(None);
}

#[derive(Clone, Copy)]
struct TestMutex;

unsafe impl RawMutex for TestMutex {
    const INIT: Self = Self;

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        let result = critical_section::with(|_| f());
        UNLOCK_HOOK.with_borrow_mut(|hook| {
            if let Some(hook) = hook {
                hook();
            }
        });
        result
    }
}

pub(super) fn handshake_poll() {
    HANDSHAKE_HOOK.with_borrow_mut(|hook| {
        if let Some(hook) = hook {
            hook();
        }
    });
}

// Keep register memory alive until the hook has been removed, including on panic.
struct HandshakeMock<'a>(PhantomData<&'a TestDevice>);

impl Drop for HandshakeMock<'_> {
    fn drop(&mut self) {
        HANDSHAKE_HOOK.with_borrow_mut(|hook| *hook = None);
    }
}

struct UnlockMock<'a>(PhantomData<&'a TestDevice>);

impl Drop for UnlockMock<'_> {
    fn drop(&mut self) {
        UNLOCK_HOOK.with_borrow_mut(|hook| *hook = None);
    }
}

fn setup() -> MutexGuard<'static, ()> {
    let guard = TEST_LOCK.lock().unwrap();
    embassy_time::MockDriver::get().reset();
    guard
}

#[test]
fn test_wait_for_success() {
    let _guard = setup();
    let mut polls = 0;
    assert!(wait_for_with_limit(
        || {
            polls += 1;
            polls == 3
        },
        3,
    ));
    assert_eq!(polls, 3);
}

#[test]
fn test_wait_for_frozen_clock() {
    let _guard = setup();
    let mut polls = 0;
    // No driver.advance(): with embassy-time enabled, its clock never advances.
    // The limit must also work while the caller holds a critical section.
    critical_section::with(|_| {
        assert!(!wait_for_with_limit(
            || {
                polls += 1;
                false
            },
            32,
        ));
    });
    assert_eq!(polls, 32);
}

#[test]
#[cfg_attr(miri, ignore)] // Native test covers the production ten-million-poll cap.
fn test_wait_for_default_limit() {
    let _guard = setup();
    let mut polls = 0;
    assert!(!wait_for(|| {
        polls += 1;
        false
    }));
    assert_eq!(polls, HANDSHAKE_POLL_LIMIT);
}

#[test]
#[cfg(feature = "embassy-time")]
fn test_wait_for_deadline() {
    let _guard = setup();
    let mut polls = 0;
    assert!(!wait_for_with_limit(
        || {
            polls += 1;
            embassy_time::MockDriver::get().advance(Duration::from_millis(1));
            false
        },
        32,
    ));
    assert_eq!(polls, 10);
}

#[test]
#[cfg(feature = "embassy-time")]
fn test_wait_for_success_before_deadline() {
    let _guard = setup();
    let mut polls = 0;
    assert!(wait_for_with_limit(
        || {
            polls += 1;
            embassy_time::MockDriver::get().advance(Duration::from_millis(1));
            polls == 2
        },
        32,
    ));
    assert_eq!(polls, 2);
}

struct TestDevice {
    registers: UnsafeCell<[u32; 0x3000 / 4]>,
    state: StateStorage<2, TestMutex>,
}

impl TestDevice {
    fn new() -> Self {
        Self {
            registers: UnsafeCell::new([0; 0x3000 / 4]),
            state: StateStorage::new(TestMutex),
        }
    }

    fn regs(&self) -> Otg {
        unsafe { Otg::from_ptr(self.registers.get().cast()) }
    }

    fn driver<'d>(&'d self, buffer: &'d mut [u8]) -> Driver<'d, TestMutex> {
        Driver::new(
            buffer,
            OtgInstance {
                regs: self.regs(),
                state: self.state.as_state(),
                fifo_depth_words: 1024,
                phy_type: PhyType::InternalFullSpeed,
                extra_rx_fifo_words: 30,
                tx_fifo_count: 2,
                calculate_trdt_fn: |_| 9,
            },
            Config::default(),
        )
    }

    fn mock_handshakes(&self, mut hook: impl FnMut(Otg) + 'static) -> HandshakeMock<'_> {
        let regs = self.regs();
        HANDSHAKE_HOOK.with_borrow_mut(|current| {
            assert!(current.is_none());
            *current = Some(Box::new(move || hook(regs)));
        });
        HandshakeMock(PhantomData)
    }

    fn mock_unlock(&self, mut hook: impl FnMut(Otg, &StateStorage<2, TestMutex>) + 'static) -> UnlockMock<'_> {
        let regs = self.regs();
        let state = &self.state as *const _;
        UNLOCK_HOOK.with_borrow_mut(|current| {
            assert!(current.is_none());
            // The guard's borrow keeps the register/state storage alive until
            // Drop removes this hook, including when an assertion panics.
            *current = Some(Box::new(move || hook(regs, unsafe { &*state })));
        });
        UnlockMock(PhantomData)
    }
}

#[test]
#[cfg_attr(miri, ignore)] // Exercises the production polling cap against stuck registers.
fn test_abort_out_endpoint_failure_releases_nak() {
    let _guard = setup();
    for nak_effective in [false, true] {
        let device = TestDevice::new();
        let r = device.regs();
        r.doepctl(1).modify(|w| w.set_epena(true));
        r.gintsts().modify(|w| w.set_goutnakeff(nak_effective));

        // RAM-backed registers never acknowledge the stop request. Exercise both
        // the NAK timeout and the subsequent endpoint-disable timeout.
        assert!(!abort_out_endpoint(r, 1));
        assert!(r.dctl().read().cgonak());
        assert_eq!(r.doepctl(1).read().epdis(), nak_effective);
    }
}

#[test]
#[cfg_attr(miri, ignore)] // Exercises the production polling cap against stuck registers.
fn test_endpoint_set_enabled_in_stop_failure() {
    let _guard = setup();
    let device = TestDevice::new();
    let r = device.regs();
    let mut buffer = [0; 64];
    let mut driver = device.driver(&mut buffer);
    let ep = driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).unwrap();
    let (mut bus, _control) = driver.start(64);
    r.diepctl(1).modify(|w| {
        w.set_epena(true);
        w.set_usbaep(true);
    });
    device.state.ep_states[1].in_enabled.store(true, Ordering::Release);
    bus.endpoint_set_enabled(ep.info.addr, false);
    assert!(!device.state.ep_states[1].in_enabled.load(Ordering::Acquire));
    assert!(!r.diepctl(1).read().usbaep());
    assert!(!r.diepctl(1).read().epdis()); // NAK was never acknowledged.
    assert!(!r.grstctl().read().txfflsh()); // Do not flush the still-active transfer.

    bus.endpoint_set_enabled(ep.info.addr, true);
    assert!(!device.state.ep_states[1].in_enabled.load(Ordering::Acquire));
    assert!(!r.diepctl(1).read().usbaep());
    assert!(!r.grstctl().read().txfflsh());
}

#[test]
#[cfg_attr(miri, ignore)] // Exercises the production polling cap against stuck registers.
fn test_endpoint_set_enabled_out_unconfirmed_stop() {
    let _guard = setup();
    let device = TestDevice::new();
    let r = device.regs();
    let mut buffer = [0; 128];
    let mut driver = device.driver(&mut buffer);
    let ep = driver.alloc_endpoint_out(EndpointType::Bulk, None, 64, 0).unwrap();
    let (mut bus, _control) = driver.start(64);
    // A timed-out disable leaves EPENA set even after clearing USBAEP.
    r.doepctl(1).modify(|w| w.set_epena(true));
    r.doeptsiz(1).write(|w| w.set_xfrsiz(17));
    bus.endpoint_set_enabled(ep.info.addr, true);
    assert!(!device.state.ep_states[1].out_enabled.load(Ordering::Acquire));
    assert!(!r.doepctl(1).read().usbaep());
    assert_eq!(r.doeptsiz(1).read().xfrsiz(), 17);
    assert!(!r.doepctl(1).read().sd0pid_sevnfrm());
    assert!(!r.doepctl(1).read().cnak());
}

#[test]
fn test_endpoint_set_enabled_active_success() {
    let _guard = setup();
    let device = TestDevice::new();
    let r = device.regs();
    let mut buffer = [0; 128];
    let mut driver = device.driver(&mut buffer);
    let ep_out = driver.alloc_endpoint_out(EndpointType::Bulk, None, 64, 0).unwrap();
    let ep_in = driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).unwrap();
    let (mut bus, _control) = driver.start(64);
    r.doepctl(1).modify(|w| w.set_epena(true));
    r.diepctl(1).modify(|w| w.set_epena(true));
    let _hardware = device.mock_handshakes(|r| {
        if r.dctl().read().sgonak() {
            r.gintsts().modify(|w| w.set_goutnakeff(true));
        }
        if r.doepctl(1).read().epdis() {
            r.doepctl(1).modify(|w| {
                w.set_epdis(false);
                w.set_epena(false);
            });
        }
        if r.diepctl(1).read().snak() {
            r.diepint(1).modify(|w| w.set_inepne(true));
        }
        if r.diepctl(1).read().epdis() {
            r.diepctl(1).modify(|w| {
                w.set_epdis(false);
                w.set_epena(false);
            });
        }
        r.grstctl().modify(|w| w.set_txfflsh(false));
    });

    bus.endpoint_set_enabled(ep_out.info.addr, true);
    assert!(device.state.ep_states[1].out_enabled.load(Ordering::Acquire));
    assert!(r.doepctl(1).read().usbaep());
    assert!(r.doepctl(1).read().epena());
    assert_eq!(r.doeptsiz(1).read().xfrsiz(), 64);
    assert!(r.dctl().read().cgonak());

    bus.endpoint_set_enabled(ep_in.info.addr, true);
    assert!(device.state.ep_states[1].in_enabled.load(Ordering::Acquire));
    assert!(r.diepctl(1).read().usbaep());
    assert!(!r.diepctl(1).read().epena());
    assert!(r.diepctl(1).read().snak());
    assert!(!r.grstctl().read().txfflsh());
}

#[test]
fn test_enabled_mirrors_published_before_unlock() {
    let _guard = setup();
    let device = TestDevice::new();
    let mut buffer = [0; 128];
    let mut driver = device.driver(&mut buffer);
    let ep_out = driver.alloc_endpoint_out(EndpointType::Bulk, None, 64, 0).unwrap();
    let (mut bus, _control) = driver.start(64);
    // At this point an ISR can run. An armed OUT endpoint must already have its
    // enabled mirror published, or the ISR would discard the first valid packet.
    let _irq = device.mock_unlock(|r, state| {
        if r.doepctl(0).read().epena() {
            assert!(state.ep_states[0].in_enabled.load(Ordering::Acquire));
            assert!(state.ep_states[0].out_enabled.load(Ordering::Acquire));
        }
        if r.doepctl(1).read().epena() {
            assert!(state.ep_states[1].out_enabled.load(Ordering::Acquire));
        }
    });
    bus.configure_endpoints();
    bus.endpoint_set_enabled(ep_out.info.addr, true);
}

#[test]
#[cfg_attr(miri, ignore)] // Exercises the production polling cap against stuck registers.
fn test_pending_write_after_flush_failure() {
    let _guard = setup();
    let device = TestDevice::new();
    let r = device.regs();
    let mut buffer = [0; 64];
    let mut driver = device.driver(&mut buffer);
    let mut ep = driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).unwrap();
    let addr = ep.info.addr;
    let (mut bus, _control) = driver.start(64);
    device.state.ep_states[1].in_enabled.store(true, Ordering::Release);
    let mut write = Box::pin(ep.write(&[0xa5; 4]));
    assert!(
        write
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
            .is_pending()
    );
    bus.endpoint_set_enabled(addr, false);
    let mut fifo_space = regs::Dtxfsts::default();
    fifo_space.set_ineptfsav(16);
    unsafe { r.dtxfsts(1).as_ptr().write(fifo_space) };
    assert_eq!(
        write.as_mut().poll(&mut Context::from_waker(Waker::noop())),
        Poll::Ready(Err(EndpointError::Disabled))
    );
    assert!(!r.diepctl(1).read().epena());
    assert_eq!(r.dieptsiz(1).read().pktcnt(), 0);
}

#[test]
#[cfg_attr(miri, ignore)] // Exercises the production polling cap against stuck registers.
fn test_setup_flush_failure() {
    let _guard = setup();
    let device = TestDevice::new();
    let r = device.regs();
    device.state.ep_states[0].in_enabled.store(true, Ordering::Release);
    device.state.ep_states[0].out_enabled.store(true, Ordering::Release);
    device.state.cp_state.setup_ready.store(true, Ordering::Release);
    r.dieptsiz(0).write(|w| w.set_pktcnt(1));
    let mut daint = regs::Daint::default();
    daint.set_oepint(1);
    unsafe { r.daint().as_ptr().write(daint) };
    r.doepint(0).write(|w| w.set_stup(true));
    r.gintsts().write(|w| {
        w.set_rxflvl(true);
        w.set_oepint(true);
    });
    let mut status = regs::Grxsts::default();
    status.set_pktstsd(vals::Pktstsd::SETUP_DATA_RX);
    status.set_bcnt(8);
    unsafe { r.grxstsp().as_ptr().write(status) };
    r.fifo(0).write_value(regs::Fifo(0x12345678));
    // Emulate RXFLVL clearing after the one popped status. TXFFLSH stays stuck.
    let _hardware = device.mock_handshakes(|r| r.gintsts().modify(|w| w.set_rxflvl(false)));
    unsafe { on_interrupt(r, &device.state.as_state()) };
    assert_eq!(device.state.cp_state.setup_data[0].load(Ordering::Relaxed), 0x12345678);
    assert_eq!(device.state.cp_state.setup_data[1].load(Ordering::Relaxed), 0x12345678);
    assert!(!device.state.cp_state.setup_ready.load(Ordering::Acquire));
    assert!(!device.state.ep_states[0].in_enabled.load(Ordering::Acquire));
    assert!(!device.state.ep_states[0].out_enabled.load(Ordering::Acquire));
}

#[test]
#[cfg_attr(miri, ignore)] // Exercises the production polling cap against stuck registers.
fn test_reset_flush_failure_discards_pending_setup() {
    let _guard = setup();
    let device = TestDevice::new();
    let r = device.regs();
    let mut buffer = [0; 64];
    let driver = device.driver(&mut buffer);
    let (mut bus, mut control) = driver.start(64);
    bus.inited = true;
    device.state.cp_state.setup_ready.store(true, Ordering::Release);
    device.state.ep_states[0].out_size.store(8, Ordering::Release);
    r.gintsts().write(|w| {
        w.set_usbrst(true);
        w.set_enumdne(true);
    });
    let mut reset = Box::pin(bus.poll());
    assert_eq!(
        reset.as_mut().poll(&mut Context::from_waker(Waker::noop())),
        Poll::Ready(Event::Reset)
    );
    drop(reset);
    assert!(!device.state.cp_state.setup_ready.load(Ordering::Acquire));
    assert_eq!(
        device.state.ep_states[0].out_size.load(Ordering::Acquire),
        EP_OUT_BUFFER_EMPTY
    );
    let mut setup = Box::pin(control.setup());
    assert!(
        setup
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
            .is_pending()
    );
}

#[test]
#[cfg_attr(miri, ignore)] // Exercises the production polling cap against stuck registers.
fn test_endpoint_set_enabled_in_flush_failure() {
    let _guard = setup();
    let device = TestDevice::new();
    let r = device.regs();
    let mut buffer = [0; 64];
    let mut driver = device.driver(&mut buffer);
    let ep = driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).unwrap();
    let (mut bus, _control) = driver.start(64);
    bus.endpoint_set_enabled(ep.info.addr, true);
    assert!(!device.state.ep_states[1].in_enabled.load(Ordering::Acquire));
    assert!(!r.diepctl(1).read().usbaep());
    assert!(!r.diepctl(1).read().sd0pid_sevnfrm());
}
