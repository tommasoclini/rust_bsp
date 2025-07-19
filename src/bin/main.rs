#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use embedded_hal::pwm::SetDutyCycle;
use embedded_io_async::Read;
use esp_hal::clock::CpuClock;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::{self, LowSpeed};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::{gpio, uart, Async};
use panic_rtt_target as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

struct BSP<'a, E> {
    ch: &'a mut dyn SetDutyCycle<Error = E>,
    counter: &'a AtomicU32,
    // reader: &'a mut R,
}

trait Holder<E> {
    fn get_bsp(&mut self) -> BSP<E>;
}

trait Configurator<E, Ctxt> {
    async fn create<'a>(ctxt: &'a mut Ctxt, cfg: Config) -> Result<impl Holder<E> + 'a, ()>;
}

struct MyHolder<'a> {
    ch: ledc::channel::Channel<'a, LowSpeed>,
    reader: uart::UartRx<'a, Async>,
    counter: &'a AtomicU32,
}

impl<'a> Holder<ledc::channel::Error> for MyHolder<'a> {
    fn get_bsp(&mut self) -> BSP<ledc::channel::Error> {
        BSP {
            ch: &mut self.ch,
            counter: self.counter,
        }
    }
}

struct MyCtxt<'a> {
    tim: ledc::timer::Timer<'a, LowSpeed>,
    uart: uart::AnyUart<'a>,
    uart_rx: gpio::AnyPin<'a>,
    pwm_pin: gpio::AnyPin<'a>,
}

struct MyConfigurator;

static COUNTER: AtomicU32 = AtomicU32::new(0);

struct Config {
    inverted: bool,
}

impl<'a> Configurator<ledc::channel::Error, MyCtxt<'a>> for MyConfigurator {
    async fn create<'b>(
        ctxt: &'b mut MyCtxt<'a>,
        cfg: Config,
    ) -> Result<impl Holder<ledc::channel::Error>, ()> {
        let (mut pwm_pin, mut uart_rx) = (ctxt.pwm_pin.reborrow(), ctxt.uart_rx.reborrow());
        if cfg.inverted {
            core::mem::swap(&mut pwm_pin, &mut uart_rx);
        }

        let mut ch = ledc::channel::Channel::new(ledc::channel::Number::Channel0, pwm_pin);
        ch.configure(ledc::channel::config::Config {
            timer: &ctxt.tim,
            duty_pct: 0,
            pin_config: ledc::channel::config::PinConfig::PushPull,
        })
        .map_err(|_| ())?;

        Ok(MyHolder {
            ch,
            reader: uart::UartRx::new(ctxt.uart.reborrow(), uart::Config::default())
                .unwrap()
                .into_async()
                .with_rx(uart_rx),
            counter: &COUNTER,
        })
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.4.0

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    let mut my_ledc = ledc::Ledc::new(peripherals.LEDC);
    my_ledc.set_global_slow_clock(ledc::LSGlobalClkSource::APBClk);

    let mut ctxt = MyCtxt {
        tim: my_ledc.timer(ledc::timer::Number::Timer0),
        pwm_pin: peripherals.GPIO10.into(),
        uart: peripherals.UART2.into(),
        uart_rx: peripherals.GPIO11.into(),
    };

    run::<_, _, MyConfigurator>(&mut ctxt).await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}

async fn run<'a, E: embedded_hal::pwm::Error, Ctxt, Cfg: Configurator<E, Ctxt>>(
    ctxt: &'a mut Ctxt,
) {
    loop {
        let mut holder = Cfg::create(
            ctxt,
            Config {
                inverted: Instant::now().as_ticks() % 2 == 0,
            },
        )
        .await
        .unwrap();
        let bsp = holder.get_bsp();

        bsp.ch.set_duty_cycle_percent(50).unwrap();
        bsp.counter.fetch_add(1, Ordering::Relaxed);

        Timer::after_secs(1).await;
    }
}
