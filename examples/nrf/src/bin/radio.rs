#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::{info, panic, unwrap};
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::radio::*;
use embassy_nrf::{interrupt, pac, peripherals, Peripherals};
use {defmt_rtt as _, panic_probe as _};
#[embassy::task]
async fn radio_task(mut radio: Radio<'static, peripherals::RADIO>) {
    let mut rx_config = RxConfig::default();
    rx_config.rx_address_0_active = true;

    let mut pkt_buf = [0u8; 130];

    info!("Begin receive");
    loop {
        match radio.receive(&rx_config, &mut pkt_buf).await {
            Ok(rssi) => {
                info!("Received packet with RSSI {} and len {}", rssi, pkt_buf[0]);
            }
            Err(e) => defmt::error!("Receiver error: {:?}", e),
        }
    }
}

#[embassy::main]
async fn main(spawner: Spawner, p: Peripherals) {
    unsafe {
        let nvmc = &*pac::NVMC::ptr();

        // UICR.APPROTECT = HwDisabled
        if *(0x10001208 as *mut u32) != 0x0000_005a {
            nvmc.config.write(|w| w.wen().wen());
            while nvmc.ready.read().ready().is_busy() {}
            core::ptr::write_volatile(0x10001208 as *mut u32, 0x0000_005a);
            while nvmc.ready.read().ready().is_busy() {}
            nvmc.config.reset();
            while nvmc.ready.read().ready().is_busy() {}
            cortex_m::peripheral::SCB::sys_reset();
        }

        // APPROTECT.DISABLE = SwDisabled
        (0x4000_0558 as *mut u32).write_volatile(0x0000_005a);
    }

    let mut cfg = Config::default();
    cfg.frequency = Frequency::new(2405).unwrap();
    cfg.mode = embassy_nrf::radio::Mode::Ieee250Kbit;
    cfg.preamble_length = PreambleLength::P32bitZero;
    cfg.payload_max = 129;
    cfg.static_length = 8;
    cfg.base_address_length = BaseAddressLength::BAL2bytes;
    cfg.endianness = Endianness::Little;
    cfg.crc_length = CrcLength::Crc2bytes;
    cfg.crc_skip_address_mode = CrcSkipAddressMode::Ieee802154;
    cfg.crc_poly = Value24Bit::new(0x0001_1021).unwrap();

    let radio = Radio::new(p.RADIO, interrupt::take!(RADIO), cfg);

    unwrap!(spawner.spawn(radio_task(radio)));
    let mut led = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);

    loop {
        led.set_high();
        info!("High!");
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        info!("Low");
        Timer::after(Duration::from_millis(500)).await;
    }
}
