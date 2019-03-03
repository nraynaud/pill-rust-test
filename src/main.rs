//! Blinks an LED
// https://github.com/ah-/anne-key/blob/master/src/main.rs
// https://rust-embedded.github.io/book/peripherals/a-first-attempt.html
// https://doc.rust-lang.org/1.30.0/book/first-edition/generics.html
// https://docs.rust-embedded.org/embedonomicon/singleton.html

// SH-HC-08 DATASHEET: https://drive.google.com/file/d/0B4urklB65vaCcEVyMm5haVVpMUk/view

// https://github.com/rust-embedded/awesome-embedded-rust#driver-crates
// https://github.com/rust-embedded/wg#the-resources-team
// https://github.com/rust-embedded/cortex-m

// https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf
// https://www.st.com/resource/en/datasheet/cd00161566.pdf

// https://github.com/MWatch/kernel/blob/rtfm-v4-hal/src/main.rs
// https://github.com/japaric/2wd/blob/master/firmware/src/main.rs
#![no_std]
#![feature(const_fn)]
#![no_main]
extern crate heapless;
//MCU: STM32F103C8T699
extern crate mpu9250;
extern crate panic_semihosting;

use core::{fmt::Write};

use cast::u32;
use cortex_m::{asm, singleton};
use either::Either;
use embedded_hal::blocking::delay::DelayMs;
use heapless::{consts::*, String, Vec};
use imu;
use mpu9250::{AccelDataRate, Dlpf, GyroTempDataRate, Marg, MargMeasurements, Mpu9250, MpuConfig};
use nb::block;
use rtfm::app;
use stm32f103xx::{EXTI, RCC, USART3 as USART3Type};
use stm32f103xx_hal::{
    afio,
    afio::MAPR,
    dma::{dma1, R, Transfer},
    gpio::{
        Alternate,
        Floating,
        gpioa,
        gpioa::PA6,
        gpiob,
        gpiob::{PB10, PB11, PB12, PB13, PB14, PB15, PB6},
        gpioc,
        gpioc::{PC13, PC14, PC15},
        Input,
        OpenDrain,
        Output,
        PullDown,
        PushPull,
    },
    prelude::*, pwm2,
    pwm2::{PWM3, Pwm3Mapping0, PWM4, Pwm4Mapping0, PwmTrait},
    rcc::{APB1, Clocks},
    serial::{Event as SerialEvent, Pins, Serial, Tx, WriteDmaImmediately},
    spi::{DmaSpi, Spi},
    time::Hertz,
};

// PINOUT
type SpiSs = PB12<Output<PushPull>>;
type SpiClock = PB13<Alternate<PushPull>>;
type SpiMiso = PB14<Input<Floating>>;
type SpiMosi = PB15<Alternate<PushPull>>;
type MpuInterrupt = PC15<Input<PullDown>>;
type UartTx = PB10<Alternate<PushPull>>;
type UartRx = PB11<Input<Floating>>;
type Led1 = PC13<Output<PushPull>>;
type Led2 = PC14<Output<PushPull>>;
type PwmLeft = PA6<Alternate<PushPull>>;
type PwmRight = PB6<Alternate<PushPull>>;


type SpiMapping = (SpiClock, SpiMiso, SpiMosi);
type SpiType = DmaSpi<SpiMapping>;
//type SpiType = Spi<SPI2, SpiMapping>;
type MpuType = Mpu9250<SpiType, SpiSs, Marg>;

type PwmMappingLeft = Pwm3Mapping0<PushPull, OpenDrain, OpenDrain, PushPull>;
type PwmTypeLeft = PWM3<PwmMappingLeft>;
type PwmMappingRight = Pwm4Mapping0<PushPull, OpenDrain, OpenDrain, PushPull>;
type PwmTypeRight = PWM4<PwmMappingRight>;

type TxType = Tx<USART3Type>;
type TxTuple = (&'static mut TxBuffType, dma1::C2, TxType);
type TxTransfer = Transfer<R, &'static mut TxBuffType, dma1::C2, TxType>;

const RX_H_SZ: usize = 20;

type TxBuffType = Vec<u8, U500>;

static mut TX_BUFFER: TxBuffType = Vec::new();

#[app(device = stm32f103xx)]
const APP: () = {
    static mut LED1: Led1 = ();
    static mut LED2: Led2 = ();
    static mut PWM_LEFT: PwmTypeLeft = ();
    static mut PWM_RIGHT: PwmTypeRight = ();
    static mut MPU: MpuType = ();
    static mut TX_EITHER: Option<Either<TxTuple, TxTransfer>> = ();
    static mut FILTER: imu::Q = ();
    static mut SPI: SpiType = ();
    static mut EXTI: EXTI = ();

    #[init]
    unsafe fn init() {
        let dbg = device.DBG;
        dbg.cr.modify(|_, w| w.dbg_sleep().set_bit());
        let rcc_open: RCC = device.RCC;
        rcc_open.apb2enr.write(|w| w.afioen().enabled());
        let mut rcc = rcc_open.constrain();
        let mut afio: afio::Parts = device.AFIO.constrain(&mut rcc.apb2);
        let mut gpioa: gpioa::Parts = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob: gpiob::Parts = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc: gpioc::Parts = device.GPIOC.split(&mut rcc.apb2);
        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .pclk2(72.mhz())
            .freeze(&mut flash.acr);
        let sysclock = clocks.sysclk();
        let dma_channels = device.DMA1.split(&mut rcc.ahb);

        let a6: PwmLeft = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
        let pins_left: PwmMappingLeft = (Some(a6), None, None, None);
        let mut pwm_left = device.TIM3.create_pwm(clocks, pins_left, &mut rcc.apb1, &mut afio.mapr);
        pwm_left.set_period(14.hz());
        pwm_left.set_duty(pwm2::Channel::_1, pwm_left.get_max_duty() / 2);
        pwm_left.enable(pwm2::Channel::_1);

        let b6: PwmRight = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let pins_right: PwmMappingRight = (Some(b6), None, None, None);
        let mut pwm_right = device.TIM4.create_pwm(clocks, pins_right, &mut rcc.apb1, &mut afio.mapr);
        pwm_right.set_period(12.hz());
        pwm_right.set_duty(pwm2::Channel::_1, pwm_right.get_max_duty() / 2);
        pwm_right.enable(pwm2::Channel::_1);

        // USART3
        let pins: (UartTx, UartRx) = (gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh), gpiob.pb11);
        let (usart, tx_channel, rx_channel) = (device.USART3, dma_channels.2, dma_channels.3, );
        let (mut serial, mut tx_channel, _rx_channel) =
            start_uart3_and_configure_hc_08(usart, pins, tx_channel, rx_channel, clocks,
                                            &mut afio.mapr, &mut rcc.apb1);
        serial.listen(SerialEvent::Tc);
        let (mut tx, _rx, _token) = serial.split();

        // SPI2
        let sck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
        let miso = gpiob.pb14;
        let mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
        let spi_mapping: SpiMapping = (sck, miso, mosi);
        let spi = Spi::spi2(device.SPI2, spi_mapping,
                            mpu9250::MODE, 500.khz(), clocks, &mut rcc.apb1);
        let mut spi = DmaSpi::new(spi, dma_channels.4, dma_channels.5);
        // stabilize SPI device
        asm::delay(seconds_to_cycles(sysclock, 0.1));
        let nss = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let mut delay = NopDelay { sysfreq: sysclock.0 };
        let mut config = MpuConfig::marg();
        config
            .gyro_temp_data_rate(GyroTempDataRate::DlpfConf(Dlpf::_0))
            .accel_data_rate(AccelDataRate::DlpfConf(Dlpf::_0))
            .sample_rate_divisor(2);
        let result = Mpu9250::marg(&mut spi, nss, &mut delay, &mut config);
        let mpu9250 = match result {
            Err(e) => {
                let mut str_buff = String::<U200>::new();
                write!(&mut str_buff, "error: {:?}\n", e).unwrap();
                tx.write_immediately(&mut tx_channel, &str_buff);
                Err(e).unwrap()
            }
            Ok(r) => r
        };
        let filter: imu::Q = imu::Q {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let _interrupt_pin: MpuInterrupt = gpioc.pc15.into_pull_down_input(&mut gpioc.crh);
        afio.exticr4.exticr4().write(|w| w.exti15().bits(0b10));
        let exti = device.EXTI;
        exti.imr.write(|w| w.mr15().set_bit());
        exti.rtsr.write(|w| w.tr15().set_bit());
        spi.change_baud_rate(18_000_000.hz());

        LED1 = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        LED2 = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
        PWM_LEFT = pwm_left;
        PWM_RIGHT = pwm_right;
        MPU = mpu9250;
        TX_EITHER = Some(Either::Left((&mut TX_BUFFER, tx_channel, tx)));
        FILTER = filter;
        SPI = spi;
        EXTI = exti;
    }

    #[allow(non_snake_case)]
    #[interrupt(resources = [LED1, EXTI, MPU, FILTER, SPI, ], priority = 2)]
    fn EXTI15_10() {
        resources.EXTI.pr.modify(|_, w| w.pr15().set_bit());
        resources.LED1.set_high();
        let a: MargMeasurements = resources.MPU.all(resources.SPI).unwrap();
        let w = imu::V {
            x: a.gyro.x,
            y: a.gyro.y,
            z: a.gyro.z,
        };
        let a_val = imu::V {
            x: a.accel.x,
            y: a.accel.y,
            z: a.accel.z,
        };
        *resources.FILTER = imu::filter_update(w, a_val, *resources.FILTER,
                                               1.0 / 4_000.0);
        resources.LED1.set_low();
    }

    #[allow(non_snake_case)]
    #[interrupt(resources = [LED2, TX_EITHER, FILTER], priority = 1)]
    fn USART3() {
        resources.LED2.set_high();
        let tx_either = resources.TX_EITHER.take();
        let maybe_tuple = match tx_either {
            Some(Either::Left(tuple)) => Some(tuple),
            Some(Either::Right(transfer)) =>
                if transfer.is_done() {
                    let (buffer, channel, mut tx) = transfer.wait();
                    block!(tx.flush()).unwrap();
                    Some((buffer, channel, tx))
                } else {
                    *resources.TX_EITHER = Some(Either::Right(transfer));
                    None
                },
            None => None
        };

        if let Some((buffer, channel, tx)) = maybe_tuple {
            let filter = resources.FILTER.lock(|filter| filter.clone());
            buffer.clear();
            let mut str_buff = String::<U200>::new();
            write!(&mut str_buff, "mpu: {:?}\n", filter).unwrap();
            // redundant copy is a bit redundant
            buffer.extend_from_slice(&str_buff.into_bytes()).unwrap();
            let transfer = tx.write_all(channel, buffer);
            *resources.TX_EITHER = Some(Either::Right(transfer));
        }
        resources.LED2.set_low();
    }

    extern "C" {
        fn EXTI1();
    }
};

fn start_uart3_and_configure_hc_08<PINS: Pins<USART3Type>>(usart: USART3Type, pins: PINS,
                                                           tx_channel: dma1::C2, rx_channel: dma1::C3,
                                                           clocks: Clocks, mapr: &mut MAPR, bus: &mut APB1)
                                                           -> (Serial<USART3Type, PINS>, dma1::C2, dma1::C3) {
    let buf = singleton!(: [[u8; RX_H_SZ]; 2] = [[0; RX_H_SZ]; 2]).unwrap();
    let baud_rates = [(b'4', 9_600.bps()), (b'5', 19_200.bps()),
        (b'6', 38_400.bps()), (b'4', 57_600.bps()), (b'8', 115_200.bps())];
    let target_baud = &baud_rates[4];
    let mut c_tx = tx_channel;
    let mut c_rx = rx_channel;
    let mut usart = usart;
    let mut pins = pins;
    let mut buf_rx = buf;
    for baud in baud_rates.iter() {
        let serial = Serial::usart3(usart, pins, mapr, baud.1, clocks, bus);
        let (mut tx, rx, token) = serial.split();
        let mut circ_buffer = rx.circ_read(c_rx, buf_rx);
        tx.write_immediately(&mut c_tx, b"AT");
        asm::delay(seconds_to_cycles(clocks.sysclk(), 1.0));
        let mut array = [0u8; RX_H_SZ * 2];
        let value = circ_buffer.peek_whole_buffer(|first, _second, len| {
            array[..len].copy_from_slice(&first[..len]);
            &array[..len]
        }).unwrap();
        if value.len() != 0 {
            if (baud.1).0 != (target_baud.1).0 {
                let mut vec: Vec<u8, U8> = Vec::<_, U8>::new();
                vec.extend_from_slice(b"AT+BAUDV").unwrap();
                vec[7] = target_baud.0;
                tx.write_immediately(&mut c_tx, &vec);
                asm::delay(seconds_to_cycles(clocks.sysclk(), 1.0));
            }
            let (_buf, rx_chan, rx) = circ_buffer.release();
            return (token.release(rx, tx), c_tx, rx_chan);
        }
        let (buf, rx_chan, rx) = circ_buffer.release();
        let (u, p) = token.release(rx, tx).release();
        c_rx = rx_chan;
        usart = u;
        pins = p;
        buf_rx = buf;
    }
    // couldn't find HC 08, configure at 115200BPS and move on
    (Serial::usart3(usart, pins, mapr, 115_200.bps(), clocks, bus), c_tx, c_rx)
}

#[derive(Debug, Copy, Clone)]
struct NopDelay {
    sysfreq: u32
}

impl DelayMs<u8> for NopDelay {
    fn delay_ms(&mut self, ms: u8) {
        asm::delay(self.sysfreq / 1000 * u32(ms));
    }
}

fn seconds_to_cycles(sysclock: Hertz, seconds: f32) -> u32 {
    (sysclock.0 as f32 * seconds) as u32
}
