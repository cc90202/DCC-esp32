#![cfg_attr(target_arch = "riscv32", no_std)]
#![cfg_attr(target_arch = "riscv32", no_main)]
#![cfg_attr(
    target_arch = "riscv32",
    deny(
        clippy::mem_forget,
        reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
        holding buffers for the duration of a data transfer."
    )
)]
#![cfg_attr(target_arch = "riscv32", deny(clippy::large_stack_frames))]

#[cfg(target_arch = "riscv32")]
mod firmware {
    use defmt::info;
    use embassy_executor::Spawner;
    use embassy_time::{Duration, Timer};
    use esp_hal::clock::CpuClock;
    use esp_println as _;

    use dcc_esp32::boot::{self, BootError};

    defmt::timestamp!("{=u64:ms}", {
        (embassy_time::Instant::now() - embassy_time::Instant::from_ticks(0)).as_millis()
    });

    #[defmt::panic_handler]
    fn defmt_panic() -> ! {
        esp_hal::system::software_reset()
    }

    #[panic_handler]
    fn panic(info: &core::panic::PanicInfo) -> ! {
        defmt::error!("PANIC: {}", defmt::Display2Format(info));
        esp_hal::system::software_reset()
    }

    // This creates a default app-descriptor required by the esp-idf bootloader.
    // For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
    esp_bootloader_esp_idf::esp_app_desc!();

    #[inline(always)]
    fn fail_fast(error: BootError) -> ! {
        defmt::error!("boot: fatal ({:?}): {}", error.action(), error.message());
        esp_hal::system::software_reset()
    }

    #[esp_rtos::main]
    async fn main(spawner: Spawner) -> ! {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);

        esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

        if let Err(error) = boot::run(spawner, peripherals).await {
            fail_fast(error);
        }

        let mut uptime_s: u32 = 0;
        loop {
            Timer::after(Duration::from_secs(10)).await;
            uptime_s = uptime_s.wrapping_add(10);
            info!("alive: uptime={}s", uptime_s);
        }
    }
}

#[cfg(not(target_arch = "riscv32"))]
fn main() {}
