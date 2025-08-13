#![no_std]
#![no_main]

use rp_pico::hal;
use hal::clocks::Clock; // for freq()
use hal::pac;
use panic_halt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
//↓これ足しました
use embedded_hal::digital::InputPin;
// use embedded_hal::digital::v2::InputPin;

use defmt_rtt as _;

mod dht20;

#[hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LED (オンボード) 確認用
    let mut led = pins.gpio25.into_push_pull_output();
    let _ = led.set_high();

    // HC-SR04 超音波センサー用 GPIO 初期化
    let mut trig = pins.gpio21.into_push_pull_output(); // Trig: GP6
    let mut echo = pins.gpio22.into_pull_down_input();      // Echo: GP7

    let mut loop_ticks: u8 = 0;          // 現在のループ
    let mut last_rh: f32 = 0.0;          // 湿度
    let mut last_temp: f32 = 0.0;        // 温度

    // ==== DHT20 I2C 初期化 (サブならI2C0 SDA=GP4, SCL=GP5、メインならI2C1 SDA=GP2, SCL=GP3) ====
    use hal::gpio::FunctionI2C;
    use hal::fugit::RateExtU32;
    //メイン移植
    // let sda = pins.gpio4.into_function::<FunctionI2C>();
    // let scl = pins.gpio5.into_function::<FunctionI2C>();
    let sda = pins.gpio2.into_function::<FunctionI2C>();
    let scl = pins.gpio3.into_function::<FunctionI2C>();
    let i2c = hal::i2c::I2C::new_controller(
        //メイン移植
        // pac.I2C0,
        pac.I2C1,
        sda,
        scl,
        100_000u32.Hz(), // 100kHz
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let mut dht = dht20::Dht20::new(i2c, timer);

    // 十分な起動待機時間
    dht.delay_mut().delay_ms(2000);  // 2秒待機
    
    // 初期化結果を詳細に確認
    match dht.init() {
        Ok(_) => {
            defmt::println!("DHT20 initialization successful");
        }
        Err(_) => {
            defmt::println!("DHT20 initialization failed");
            
            // 再試行
            dht.delay_mut().delay_ms(1000);
            match dht.init() {
                Ok(_) => {
                    defmt::println!("DHT20 retry initialization successful");
                }
                Err(_) => {
                    defmt::println!("DHT20 retry initialization failed");
                }
            }
        }
    }

    // 湿度に応じて点滅速度を変える: 0%→1000ms, 100%→100ms
    // const MAX_DELAY_MS: u32 = 1000;
    // const MIN_DELAY_MS: u32 = 100;

    loop {
        // --- DHT20 温湿度センサー測定 ---
        let _ = led.set_low();
        match dht.read() {
            Ok((rh, t)) => { last_rh = rh; last_temp = t; }
            Err(_) => { defmt::println!("read error"); }
        }
        //let rh_clamped = if last_rh < 0.0 { 0.0 } else if last_rh > 100.0 { 100.0 } else { last_rh };
        // let span = (MAX_DELAY_MS - MIN_DELAY_MS) as f32;
        // let delay_ms = (MAX_DELAY_MS as f32 - span * (rh_clamped / 100.0)) as u32;
        // defmt::println!("loop:{} RH:{}% T:{}C delay:{}ms", loop_ticks, last_rh, last_temp, delay_ms);
        defmt::println!("loop:{} RH:{}% T:{}C", loop_ticks, last_rh, last_temp);
        loop_ticks = loop_ticks.wrapping_add(1);
        let _ = led.set_low();
        dht.delay_mut().delay_ms(100);
        let _ = led.set_high();

        // --- HC-SR04 超音波センサー測定 ---
        let _ = trig.set_low();
        dht.delay_mut().delay_us(2);
        let _ = trig.set_high();
        dht.delay_mut().delay_us(12);
        let _ = trig.set_low();

        // Echo立ち上がり待ち（タイムアウト付き）
        let timeout_us = 30_000;
        let start = dht.delay_mut().get_counter();
        while echo.is_low().unwrap() {
            if dht.delay_mut().get_counter().ticks() - start.ticks() > timeout_us as u64 {
                defmt::println!("Echo timeout (no pulse)");
                break;
            }
        }
        let pulse_start = dht.delay_mut().get_counter();
        while echo.is_high().unwrap() {
            if dht.delay_mut().get_counter().ticks() - pulse_start.ticks() > timeout_us as u64 {
                defmt::println!("Echo timeout (too long)");
                break;
            }
        }
        let pulse_end = dht.delay_mut().get_counter();
        let pulse_width_us = (pulse_end.ticks() - pulse_start.ticks()) as u32;
        let distance_cm = pulse_width_us as f32 * 0.0343 / 2.0;
        defmt::println!("Distance: {} cm", distance_cm);

        // 測定周期（2秒間隔）
        dht.delay_mut().delay_ms(2000);
    }
}