//! DSIHOST Physical Layer (PHY)

use super::{DsiHost, Instance};
#[cfg(dsihost_v1)]
use crate::rcc::get_freqs;

/// Number of DSI PHY data lanes
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum DsiHostPhyLanes {
    /// One data lane
    One = 0,
    /// Two data lanes
    Two = 1,
}

/// DSI PHY Configuration
pub struct DsiHostPhyConfig {
    /// Number of DSI PHY lanes
    pub lanes: DsiHostPhyLanes,

    /// Stop wait time. Minimum wait period to request a high speed transmission after the stop state
    pub stop_wait_time: u8,

    /// D-PHY automatic clock lane control. Stops providing clock automatically.
    pub acr: bool,

    /// Enable CRC RX enable
    pub crc_rx: bool,

    /// Enable ECC RX enable
    pub ecc_rx: bool,

    /// EoTp RX enable
    pub eotp_rx: bool,

    /// EoTp TX enable
    pub eotp_tx: bool,

    /// Bus Turnaround enable
    pub bta: bool,

    /// Clock high speed to low power timing
    pub clock_hs2lp: u16,

    /// Clock low power to high speed timing
    pub clock_lp2hs: u16,

    /// Data high speed to low power timing
    #[cfg(dsihost_v1)]
    pub data_hs2lp: u8,

    /// Data high speed to low power timing
    #[cfg(dsihost_u5)]
    pub data_hs2lp: u16,

    /// Data low power to high speed timing
    #[cfg(dsihost_v1)]
    pub data_lp2hs: u8,

    /// Data low power to high speed timing
    #[cfg(dsihost_u5)]
    pub data_lp2hs: u16,

    /// Data maximum read time
    pub data_mrd: u16,
}

impl<'d, T: Instance> DsiHost<'d, T> {
    /// Initialize DSI D-PHY
    pub fn phy_init(&mut self, config: &DsiHostPhyConfig) {
        let lane_clock = T::frequency();

        // Timeout clock: target ~= 20 MHz. Choose ceiling below 20MHz
        const ESCAPE_TARGET_HZ: u32 = 20_000_000;
        let escape_div = ((lane_clock.0 + (ESCAPE_TARGET_HZ - 1)) / ESCAPE_TARGET_HZ) as u8; // ceil
        assert!(
            escape_div > 1 && escape_div <= 32,
            "DSI escape clock divider out of range"
        );
        let tx_escape_clock = lane_clock / escape_div;

        // Timeout clock: target ~= 1 MHz
        const TIMEOUT_TARGET_HZ: u32 = 1_000_000;
        let timeout_div = ((lane_clock.0 + (TIMEOUT_TARGET_HZ / 2)) / TIMEOUT_TARGET_HZ) as u8;
        assert!(timeout_div > 0, "DSI timeout clock divider out of range");
        let timeout_clock = lane_clock / timeout_div;

        #[cfg(feature = "defmt")]
        {
            debug!(
                "DSI lane byte clock: {} tx escape clock: {} timeout clock: {}",
                lane_clock, tx_escape_clock, timeout_clock
            );
        }

        self.lane_byte_clock = Some(lane_clock).into();
        self.tx_escape_clock = Some(tx_escape_clock).into();
        self.timeout_clock = Some(timeout_clock).into();

        #[cfg(dsihost_u5)]
        {
            // 44.14.1 item 8 enable dsi host
            T::regs().cr().modify(|w| w.set_en(true));
        }

        // item 9?
        T::regs().ccr().modify(|w| {
            w.set_tockdiv(timeout_div);
            w.set_txeckdiv(escape_div);
        });

        // item 10
        #[cfg(dsihost_u5)]
        {
            // (44.14.2 Configuring the D-PHY parameters)

            // * the slew rate for the clock lane through register DSI_DPCSRC
            // 0x0e 80-750 Mbit
            T::regs().dpcsrcr().modify(|w| w.set_src(0x0e));

            // * the slew rate for the data lanes through DSI_DPDL0SRC and DSIDPDL1SRC
            T::regs().dpdl0srcr().modify(|w| w.set_src(0x0e));
            T::regs().dpdl1srcr().modify(|w| w.set_src(0x0e));

            // TODO: is that 500Mhz information in 'config'? (that would allow 'match' intervals, and avoid a hardcoded value)
            // 450-510 MHz
            let band: u8 = 0b01000;

            // * the frequency band for the clock lane through DSI_DPCLBCR
            T::regs().dpcbcr().modify(|w| w.set_bc(band));
            // * the frequency band for the data lanes through DSI_DPDL0BCR and DSI_DPBL1BCR
            T::regs().dpdl0bcr().modify(|w| w.set_bc(band));
            T::regs().dpdl1bcr().modify(|w| w.set_bc(band));

            // Configuring the D-PHY parameters in the DSI Host

            // * Number of data lanes in the DSI_PCONFR register
            T::regs().pconfr().modify(|w| {
                w.set_nl(config.lanes as u8);
            });

            // * Automatic clock lane control (ACR) in the DSI_CLCR register
            // * Clock control (DPCC) in the DSI_CLCR register
            // ^^ WRONG, this cannot be done here! Setting 'DPCC' here results in failure to get 'psr.pssc' in the loop later..

            // * Time for LP/HS and HS/LP transitions for both clock lane and data lanes in DSI_CLTCR and DSI_DLTCR registers
            T::regs().cltcr().modify(|w| {
                w.set_hs2lp_time(config.clock_hs2lp);
                w.set_lp2hs_time(config.clock_lp2hs);
            });
            T::regs().dltcr().modify(|w| {
                #[cfg(dsihost_v1)]
                w.set_mrd_time(config.data_mrd);
                w.set_hs2lp_time(config.data_hs2lp);
                w.set_lp2hs_time(config.data_lp2hs);
            });

            // * Stop wait time in the DSI_PCONFR register
            T::regs().pconfr().modify(|w| {
                w.set_sw_time(config.stop_wait_time);
            });
        }

        #[cfg(not(dsihost_u5))]
        self.phy_clock_enable(true);

        #[cfg(dsihost_u5)]
        {
            // 44.14.1 item 11 enable d-phy digital section
            T::regs().pctlr().modify(|w| {
                w.set_den(true);
            });
        }

        // 44.14.1 item 12
        #[cfg(dsihost_u5)]
        {
            // Configure the band control (BC fields) and the slew rate (SRC fields) for the clock lane
            // and the data lanes in DSI_DPCBCR, DSI_DPDLxBCD, DSI_DPCSRCR, and
            // DSI_DPDLxSRCR


            // dpdl0hsocr/dpdl1hsocr
            unsafe {
                // TODO: avoid hardcoded value
                // 450-510 MHz
                let hsprpo: u8 = 0b0010;
                {
                    let dpdl0hsocr = T::regs().as_ptr().add(0x0c5c) as *mut u32;
                    // read-first to not-to-change reserved values
                    const HSPRPO_START: usize = 4;
                    const HSPRPO_MASK: u32 = 0b1111;
                    let mut value = core::ptr::read_volatile(dpdl0hsocr);
                    value = (value & !(HSPRPO_MASK << HSPRPO_START)) | (((hsprpo as u32) & HSPRPO_MASK) << HSPRPO_START);
                    core::ptr::write_volatile(dpdl0hsocr, value);
                }
                {
                    let dpdl1hsocr = T::regs().as_ptr().add(0x0cf4) as *mut u32;
                    // read-first to not-to-change reserved values
                    const HSPRPO_START: usize = 4;
                    const HSPRPO_MASK: u32 = 0b1111;
                    let mut value = core::ptr::read_volatile(dpdl1hsocr);
                    value = (value & !(HSPRPO_MASK << HSPRPO_START)) | (((hsprpo as u32) & HSPRPO_MASK) << HSPRPO_START);
                    core::ptr::write_volatile(dpdl1hsocr, value);
                }
            }
        }

        // 44.14.1 item 13
        #[cfg(dsihost_u5)]
        {
            T::regs().pctlr().modify(|w| {
                // enable clock lanes
                w.set_cke(true);
            });
        }


        // FAILS: without it pss0 and pss1 get true (pssc is false), with it all 3 are false
        /*
        // switch from PLL3 to D-PHY PLL
        crate::pac::RCC.ccipr2().modify(|w| {
            w.set_dsisel(crate::pac::rcc::vals::Dsisel::DsiPhyDiv8)
        });
         */

        #[cfg(dsihost_v1)]
        {
            let clocks = unsafe { get_freqs() };
            let phy_clock = clocks
                .dsi_phy
                .to_hertz()
                .expect("DSI PHY clock must be enabled before init");

            // Calculate UIX4
            let uix4 = (4_000_000_000u64) / phy_clock.0 as u64;
            let uix4 = uix4.max(1).min(63) as u8;
            T::regs().wpcr0().modify(|w| {
                w.set_uix4(uix4);
            });
        }

        T::regs().pcr().modify(|w| {
            w.set_crcrxe(config.crc_rx);
            w.set_eccrxe(config.ecc_rx);
            w.set_etrxe(config.eotp_rx);
            w.set_ettxe(config.eotp_tx);
            w.set_btae(config.bta);
        });


        // 44.14.1 item 15
        #[cfg(dsihost_u5)]
        {
            #[cfg(feature = "defmt")]
            defmt::debug!("Waiting for psr");
            use core::ops::Add;
            //let wait_end = embassy_time::Instant::now().add(embassy_time::Duration::from_millis(200));
            // NOTE: even extremely long 2 second timeout does not work
            let wait_end = embassy_time::Instant::now().add(embassy_time::Duration::from_millis(2000));
            loop {
                let psr = T::regs().psr().read();
                // shows all 3 false
                defmt::debug!("pssc: {}, pss0: {}, pss1: {}", psr.pssc(), psr.pss0(), psr.pss1());

                if psr.pssc() && psr.pss0() && (psr.pss1() || matches!(config.lanes, DsiHostPhyLanes::One)) {
                    break;
                }
                if wait_end < embassy_time::Instant::now() {
                    panic!("Timed out waiting for psr");
                }
                // 1ms before reading again (blocking)
                crate::wait::block_for_us(1000);
            }
            #[cfg(feature = "defmt")]
            defmt::debug!("Done waiting for psr");

            // DEBUG
            {
                let s = T::regs().wisr().read();
                let p = T::regs().psr().read();
                defmt::debug!("phy_init after loop: post-init: PLLLS={} PLLUIF={} PSSC={} PSS0={} PSS1={} UANC={} UAN0={} UAN1={}",
                    s.pllls(), s.plluif(), p.pssc(), p.pss0(), p.pss1(), p.uanc(), p.uan0(), p.uan1());
            }

            // 44.14.1 item 16
            T::regs().cr().modify(|w|w.set_en(false));

            // part of item 10 moved here (not sure about the correct location)
            T::regs().clcr().modify(|w| {
                w.set_dpcc(true);
                w.set_acr(config.acr);
            });

            // DSISEL? (no change noticed, disabled for now - clock shloud be same anyway...)
            /*
            {
                let w = T::regs().wrpcr().read();
                let s = T::regs().wisr().read();
                defmt::debug!("DSI PLL pre-switch: PLLEN={} PLLLS={} PLLUIF={}", w.pllen(), s.pllls(), s.plluif());
                crate::pac::RCC.ccipr2().modify(|w| {
                    w.set_dsisel(crate::pac::rcc::vals::Dsisel::DsiPhyDiv8)
                });
            }*/

            // DEBUG
            {
                let s = T::regs().wisr().read();
                let p = T::regs().psr().read();
                defmt::debug!("phy_init end of function: post-init: PLLLS={} PLLUIF={} PSSC={} PSS0={} PSS1={} UANC={} UAN0={} UAN1={}",
                    s.pllls(), s.plluif(), p.pssc(), p.pss0(), p.pss1(), p.uanc(), p.uan0(), p.uan1());
            }
        }
    }

    /// Enable the PHY clocks and digital domain
    fn phy_clock_enable(&mut self, enable: bool) {
        T::regs().pctlr().modify(|w| {
            // Enable D-PHY clock lane module
            w.set_cke(enable);
            // Enable digital section of D-PHY
            w.set_den(enable);
        });
    }
}
