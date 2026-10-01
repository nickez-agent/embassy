use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Waker};

use embassy_usb_driver::EndpointAddress;

use super::*;

struct TestHost<M: RawMutex + Copy = CriticalSectionRawMutex> {
    registers: UnsafeCell<[u32; 0x2000 / 4]>,
    state: HostStateStorage<1, M>,
}

impl TestHost {
    fn new() -> Self {
        Self::with_mutex(CriticalSectionRawMutex::new())
    }
}

impl<M: RawMutex + Copy> TestHost<M> {
    fn with_mutex(mutex: M) -> Self {
        Self {
            registers: UnsafeCell::new([0; 0x2000 / 4]),
            state: HostStateStorage::new(mutex),
        }
    }

    fn regs(&self) -> Otg {
        // The test backing contains the channel registers and RX FIFO register.
        unsafe { Otg::from_ptr(self.registers.get().cast()) }
    }

    fn allocator(&self) -> OtgHostAllocator<'_, M> {
        OtgHost::new(OtgHostInstance {
            regs: self.regs(),
            state: self.state.as_host_state(),
            fifo_depth_words: 1024,
            phy_type: PhyType::InternalHighSpeed,
        })
        .allocator()
    }

    fn pipe<T: pipe::Type>(&self, max_packet_size: u16) -> Channel<'_, T, pipe::In, M> {
        self.allocator()
            .alloc_pipe(1, &endpoint::<T>(max_packet_size), None)
            .unwrap()
    }

    fn receive(&self, len: u16, byte: u8) {
        let r = self.regs();
        r.fifo(0).write_value(regs::Fifo(u32::from_ne_bytes([byte; 4])));
        r.hctsiz(0).modify(|w| w.set_pktcnt(w.pktcnt().saturating_sub(1)));
        let mut status = regs::Grxsts::default();
        status.set_pktstsh(vals::Pktstsh::IN_DATA_RX);
        status.set_bcnt(len);
        // RAM-backed registers cannot model the pop side effects of GRXSTSP or
        // the FIFO. Feed one popped status to the same helper used by the ISR.
        self.state.mutex.lock(|| unsafe {
            read_rx_packet(r, &self.state.channels, status);
        });
    }

    fn finish(&self, events: u16) {
        // Emulate hardware stopping the channel before reporting its result.
        self.regs().hcchar(0).modify(|w| w.set_chena(false));
        // Model the ISR acknowledging hardware flags when it publishes these events.
        self.regs().hcint(0).write_value(regs::Hcint(0));
        self.state.channels[0].result.fetch_or(events, Ordering::Release);
    }

    fn rx_count(&self) -> usize {
        self.state
            .mutex
            .lock(|| unsafe { *self.state.channels[0].rx_count.get() })
    }
}

fn endpoint<T: pipe::Type>(max_packet_size: u16) -> EndpointInfo {
    EndpointInfo {
        addr: EndpointAddress::from(0x81),
        ep_type: T::ep_type(),
        max_packet_size,
        interval_ms: 1,
    }
}

fn poll<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}

#[test]
fn test_request_in_cancel() {
    let host = TestHost::new();
    let mut channel = host.pipe::<pipe::Interrupt>(64);
    let mut buffer = vec![0; 64];
    let mut request = Box::pin(channel.request_in(&mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    drop(request);
    drop(buffer);

    // Miri checks that the delayed ISR never touches the freed caller buffer.
    host.receive(64, 0xa5);
    host.finish(EV_XFRC);

    let mut replacement = [0; 64];
    let mut request = Box::pin(channel.request_in(&mut replacement));
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.rx_count(), 0);
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA1.to_bits());
    host.receive(7, 0x3c);
    host.finish(EV_XFRC);
    assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(7)));
    drop(request);
    assert_eq!(&replacement[..7], &[0x3c; 7]);
    assert_eq!(&replacement[7..], &[0; 57]);
}

#[test]
fn test_request_in_forget_and_release_channel() {
    let host = TestHost::new();
    let mut channel = host.pipe::<pipe::Interrupt>(64);
    let mut buffer = vec![0; 64];
    let mut request = Box::pin(channel.request_in(&mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    // Leak the actual pinned future, not just a Pin<&mut _> whose pointee would
    // still be dropped. Run Miri with -Zmiri-ignore-leaks for this regression.
    core::mem::forget(request);
    drop(buffer);
    host.receive(64, 0xa5);
    drop(channel);

    // A late packet must be discarded without re-arming the released channel.
    host.regs().hcchar(0).modify(|w| w.set_chena(false));
    host.regs().hctsiz(0).modify(|w| w.set_pktcnt(2));
    host.receive(64, 0xff);
    assert!(!host.regs().hcchar(0).read().chena());

    let mut channel = host.pipe::<pipe::Interrupt>(64);
    let mut replacement = [0; 64];
    let mut request = Box::pin(channel.request_in(&mut replacement));
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.rx_count(), 0);
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA0.to_bits());
    host.receive(64, 0x3c);
    host.finish(EV_XFRC);
    assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(64)));
    drop(request);
    assert_eq!(replacement, [0x3c; 64]);
}

#[test]
fn test_request_in_bulk_multiple_staging_buffers() {
    let host = TestHost::new();
    let mut channel = host.pipe::<pipe::Bulk>(64);
    let mut buffer = [0; RX_BUFFER_SIZE + 65];
    let mut request = Box::pin(channel.request_in(&mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().pktcnt(), 16);
    for _ in 0..16 {
        host.receive(64, 0xa5);
    }
    host.finish(EV_XFRC);
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().xfrsiz(), 65);
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA0.to_bits());
    host.receive(64, 0x3c);
    host.receive(1, 0xff);
    host.finish(EV_XFRC);
    assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(RX_BUFFER_SIZE + 65)));
    drop(request);
    assert_eq!(&buffer[..RX_BUFFER_SIZE], &[0xa5; RX_BUFFER_SIZE]);
    assert_eq!(&buffer[RX_BUFFER_SIZE..RX_BUFFER_SIZE + 64], &[0x3c; 64]);
    assert_eq!(buffer[RX_BUFFER_SIZE + 64], 0xff);
    assert!(!channel.data_toggle);
}

#[test]
fn test_request_in_forget_and_reuse_channel() {
    let host = TestHost::new();
    let mut channel = host.pipe::<pipe::Bulk>(64);
    let mut buffer = vec![0; 128];
    let mut request = Box::pin(channel.request_in(&mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    core::mem::forget(request);
    drop(buffer);
    host.receive(64, 0xa5);
    host.finish(EV_NAK | EV_CHH);

    let mut replacement = [0; 64];
    let mut request = Box::pin(channel.request_in(&mut replacement));
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA1.to_bits());
    host.receive(64, 0x3c);
    host.finish(EV_XFRC);
    assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(64)));
    drop(request);
    assert_eq!(replacement, [0x3c; 64]);
    assert!(!channel.data_toggle);
}

#[test]
fn test_request_in_bulk_nak_after_packet() {
    let host = TestHost::new();
    let mut channel = host.pipe::<pipe::Bulk>(64);
    let mut buffer = [0; 128];
    let mut request = Box::pin(channel.request_in(&mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    host.receive(64, 0xa5);
    host.finish(EV_NAK | EV_CHH);
    assert!(poll(request.as_mut()).is_pending()); // Yield between retries.
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().xfrsiz(), 64);
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA1.to_bits());
    host.receive(64, 0x3c);
    host.finish(EV_XFRC);
    assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(128)));
    drop(request);
    assert_eq!(&buffer[..64], &[0xa5; 64]);
    assert_eq!(&buffer[64..], &[0x3c; 64]);
    assert!(!channel.data_toggle);
}

#[test]
fn test_request_in_cancel_between_staging_buffers() {
    let host = TestHost::new();
    let mut channel = host.pipe::<pipe::Bulk>(1024);
    let mut buffer = vec![0; 3 * RX_BUFFER_SIZE];
    let mut request = Box::pin(channel.request_in(&mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    host.receive(1024, 0xa5);
    host.finish(EV_XFRC);
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA1.to_bits());
    drop(request);
    drop(buffer);
    // A second packet is acknowledged after cancellation. Both must count.
    host.receive(1024, 0xa5);
    host.finish(EV_XFRC);
    let mut replacement = [0; 1024];
    let mut request = Box::pin(channel.request_in(&mut replacement));
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA0.to_bits());
    host.receive(7, 0x3c);
    host.finish(EV_XFRC);
    assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(7)));
    drop(request);
    assert!(channel.data_toggle);
}

#[test]
fn test_request_in_bulk_short_packet_and_zlp() {
    for short_len in [0, 7] {
        let host = TestHost::new();
        let mut channel = host.pipe::<pipe::Bulk>(64);
        let mut buffer = [0; 2 * RX_BUFFER_SIZE];
        let mut request = Box::pin(channel.request_in(&mut buffer));
        assert!(poll(request.as_mut()).is_pending());
        host.receive(64, 0xa5);
        host.receive(short_len, 0x3c);
        host.finish(EV_XFRC);
        assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(64 + usize::from(short_len))));
        drop(request);
        assert_eq!(&buffer[..64], &[0xa5; 64]);
        assert!(!channel.data_toggle); // Both a short data packet and a ZLP consume a PID.
    }
}

#[test]
fn test_request_in_periodic_maximum_packet() {
    let host = TestHost::new();
    let mut channel = host.pipe::<pipe::Interrupt>(1024);
    let mut buffer = [0; 2 * RX_BUFFER_SIZE];
    let mut request = Box::pin(channel.request_in(&mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().pktcnt(), 1);
    host.receive(1024, 0xa5);
    host.finish(EV_XFRC);
    assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(1024)));
    drop(request);
    assert_eq!(&buffer[..1024], &[0xa5; 1024]);
    assert_eq!(&buffer[1024..], &[0; 1024]);
    assert!(channel.data_toggle);
}

#[test]
fn test_control_in_multiple_packets() {
    let host = TestHost::new();
    let mut channel = host.pipe::<pipe::Control>(64);
    let mut buffer = [0; 128];
    let setup = [0x80, 6, 0, 1, 0, 0, 128, 0];
    let mut request = Box::pin(channel.control_in(&setup, &mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::SETUP.to_bits());
    host.finish(EV_XFRC);
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA1.to_bits());
    host.receive(64, 0xa5);
    host.finish(EV_XFRC);
    assert!(poll(request.as_mut()).is_pending());
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA0.to_bits());
    host.receive(7, 0x3c);
    host.finish(EV_XFRC);
    assert!(poll(request.as_mut()).is_pending());
    assert!(!host.regs().hcchar(0).read().epdir());
    assert_eq!(host.regs().hctsiz(0).read().xfrsiz(), 0);
    assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA1.to_bits());
    host.finish(EV_XFRC);
    assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(71)));
    drop(request);
    assert_eq!(&buffer[..64], &[0xa5; 64]);
    assert_eq!(&buffer[64..71], &[0x3c; 7]);
}

#[test]
fn test_alloc_pipe_invalid_packet_size() {
    let host = TestHost::new();
    for mps in [0, 1025, u16::MAX] {
        assert!(matches!(
            host.allocator()
                .alloc_pipe::<pipe::Bulk, pipe::In>(1, &endpoint::<pipe::Bulk>(mps), None),
            Err(HostError::InvalidDescriptor)
        ));
    }
}

#[derive(Default)]
struct LatePacket {
    regs: core::cell::Cell<Option<Otg>>,
    channel: core::cell::Cell<*const ChannelState>,
    packet: core::cell::Cell<Option<(u16, bool)>>,
}

#[derive(Clone, Copy)]
struct ReceiveOnUnlock<'a>(Option<&'a LatePacket>);

// SAFETY: The critical section provides the same mutual exclusion as the driver mutex.
// The hook runs synchronously under that lock. Tests arm it only for driver RX-state
// operations whose closures do not return references into the channel storage.
unsafe impl RawMutex for ReceiveOnUnlock<'_> {
    const INIT: Self = Self(None);

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        CriticalSectionRawMutex::new().lock(|| {
            let result = f();
            if let Some(hook) = self.0 {
                if let Some((len, stall)) = hook.packet.take() {
                    let r = hook.regs.get().unwrap();
                    r.fifo(0).write_value(regs::Fifo(0x3c3c3c3c));
                    let mut status = regs::Grxsts::default();
                    status.set_pktstsh(vals::Pktstsh::IN_DATA_RX);
                    status.set_bcnt(len);
                    // The test keeps this channel in place until the armed hook has run.
                    // Model a packet acknowledged before the hardware finishes stopping.
                    unsafe { read_rx_packet(r, core::slice::from_raw_parts(hook.channel.get(), 1), status) };
                    r.hcchar(0).modify(|w| w.set_chena(false));
                    r.hcint(0).write(|w| w.set_stall(stall));
                }
            }
            result
        })
    }
}

#[test]
fn test_request_in_nak_preserves_packet_received_during_stop() {
    for late_len in [0, 7, 64] {
        let hook = LatePacket::default();
        let host = TestHost::with_mutex(ReceiveOnUnlock(Some(&hook)));
        let mut channel = host.pipe::<pipe::Bulk>(64);
        let mut buffer = vec![0; if late_len == 64 { 192 } else { 128 }];
        let mut request = Box::pin(channel.request_in(&mut buffer));
        assert!(poll(request.as_mut()).is_pending());
        host.receive(64, 0xa5);
        host.state.channels[0].result.fetch_or(EV_NAK, Ordering::Release);

        // The next RX-state lock marks the chunk as stopping. Deliver one last
        // accepted packet before the halt completes, without racing fake MMIO.
        hook.regs.set(Some(host.regs()));
        hook.channel.set(&host.state.channels[0]);
        hook.packet.set(Some((late_len, false)));
        if late_len == 64 {
            assert!(poll(request.as_mut()).is_pending()); // Yield before retrying.
            assert!(poll(request.as_mut()).is_pending());
            // Both acknowledged packets count before the remaining chunk is programmed.
            assert_eq!(host.regs().hctsiz(0).read().dpid(), vals::Dpid::DATA0.to_bits());
            assert_eq!(host.regs().hctsiz(0).read().xfrsiz(), 64);
            host.receive(64, 0x5a);
            host.finish(EV_XFRC);
            assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(192)));
        } else {
            assert_eq!(poll(request.as_mut()), Poll::Ready(Ok(64 + usize::from(late_len))));
        }
        drop(request);
        assert_eq!(&buffer[..64], &[0xa5; 64]);
        assert_eq!(
            &buffer[64..64 + usize::from(late_len)],
            &vec![0x3c; usize::from(late_len)]
        );
        assert_eq!(channel.data_toggle, late_len == 64);
        if late_len == 64 {
            assert_eq!(&buffer[128..], &[0x5a; 64]);
        }
        assert!(!host.regs().hcchar(0).read().chena());
    }
}

#[test]
fn test_request_in_preserves_error_received_during_stop() {
    let hook = LatePacket::default();
    let host = TestHost::with_mutex(ReceiveOnUnlock(Some(&hook)));
    let mut channel = host.pipe::<pipe::Bulk>(64);
    let mut buffer = [0; 128];
    let mut request = Box::pin(channel.request_in(&mut buffer));
    assert!(poll(request.as_mut()).is_pending());
    host.receive(64, 0xa5);
    host.state.channels[0].result.fetch_or(EV_NAK, Ordering::Release);
    hook.regs.set(Some(host.regs()));
    hook.channel.set(&host.state.channels[0]);
    hook.packet.set(Some((7, true)));
    assert_eq!(poll(request.as_mut()), Poll::Ready(Err(PipeError::Stall)));
    drop(request);
    assert_eq!(buffer, [0; 128]);
}
