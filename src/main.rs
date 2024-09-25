#![no_std]
#![no_main]

extern crate alloc;

use alloc::{boxed::Box, rc::Rc, vec::Vec};
use core::{cell::RefCell, mem::MaybeUninit};
use embedded_hal_bus::spi::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{self, Io, Output},
    peripherals::Peripherals,
    prelude::*,
    spi::{self, master::Spi},
    system::SystemControl,
    timer::systimer::SystemTimer,
};
use gc9a01::{
    mode::{BufferedGraphics, DisplayConfiguration},
    prelude::{DisplayDefinition, DisplayResolution240x240, DisplayRotation, WriteOnlyDataCommand},
    Gc9a01, SPIDisplayInterface,
};
use slint::platform::software_renderer::MinimalSoftwareWindow;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

slint::include_modules!();

#[entry]
fn main() -> ! {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let pins = Io::new(peripherals.GPIO, peripherals.IO_MUX).pins;

    let sck = pins.gpio10;
    let mosi = pins.gpio11;
    let cs = pins.gpio9;
    let dc = pins.gpio8;
    let reset = pins.gpio14;
    let backlight = pins.gpio2;

    let cs_output = Output::new(cs, gpio::Level::Low);
    let dc_output = Output::new(dc, gpio::Level::Low);
    Output::new(backlight, gpio::Level::High);
    let mut reset_output = Output::new(reset, gpio::Level::Low);

    let spi = Spi::new(peripherals.SPI2, 40u32.MHz(), spi::SpiMode::Mode0, &clocks).with_pins(
        Some(sck),
        Some(mosi),
        gpio::NO_PIN,
        gpio::NO_PIN,
    );

    let spi_bus = RefCell::new(spi);
    let spi_device = RefCellDevice::new_no_delay(&spi_bus, cs_output).unwrap();
    let interface = SPIDisplayInterface::new(spi_device, dc_output);

    let mut display_driver = Gc9a01::new(
        interface,
        DisplayResolution240x240,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics();

    display_driver.clear_fit().unwrap();
    display_driver.reset(&mut reset_output, &mut delay).ok();
    display_driver.init(&mut delay).ok();
    display_driver.flush().unwrap();

    log::info!("Driver configured!");

    let window = MinimalSoftwareWindow::new(Default::default());
    window.set_size(slint::PhysicalSize::new(240 as _, 240 as _));

    slint::platform::set_platform(Box::new(EspBackend {
        window: window.clone(),
    }))
    .unwrap();

    let mut draw_buffer = DrawBuffer {
        display: display_driver,
        buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 240],
    };

    let _app_window = AppWindow::new().unwrap();

    loop {
        slint::platform::update_timers_and_animations();

        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut draw_buffer);
        });

        if window.has_active_animations() {
            continue;
        }
    }
}

struct EspBackend {
    window: Rc<MinimalSoftwareWindow>,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            SystemTimer::now() / (SystemTimer::ticks_per_second() / 1000),
        )
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<I: WriteOnlyDataCommand, D: DisplayDefinition>
    slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Gc9a01<I, D, BufferedGraphics<D>>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        _line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        // Endianness issue here inverts the colors
        self.display
            .send_line(&buffer.iter().map(|&x| x.0.to_be()).collect::<Vec<u16>>())
            .unwrap();
    }
}
