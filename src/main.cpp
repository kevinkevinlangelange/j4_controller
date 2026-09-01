//******************************************************************************
//       file name:  j4_controller.ino
//     v0_1 created:  2023-11-08 @ 1209 CST  -KL
//     v0_6 created:  2023-11-16 @ 2226 CST  -KL
//   v0_6_4 created:  2026-05-20 @ 0700 CDT  -KL
//          v0_6_11:  2026-05-31 @ 1752 CDT  -KL
//          v0_6_12:  2026-06-02 @ 1045 CDT  -KL
//          v0_6_13:  2026-06-07  -KL
//          v0_6_14:  2026-06-10  -KL
//          v0_6_15:  2026-06-15  -KL
//          v0_6_16:  2026-06-16  -KL
//          v0_6_17:  2026-06-17  -KL
//          v0_6_18:  2026-07-02  -KL
//          v0_6_19:  2026-07-03  -KL
//          v0_6_20:  2026-07-04  -KL
//          v0_6_21:  2026-07-08  -KL
//          v0_6_22:  2026-07-12  -KL
//          v0_6_23:  2026-07-14  -KL
//          v0_6_24:  2026-08-21  -KL
//          v0_6_25:  2026-08-23  -KL
//          v0_6_26:  2026-08-24  -KL
//          v0_6_27:  2026-08-29  -KL
//          v0_6_28:  2026-08-29  -KL
//          v0_6_29:  2026-08-29  -KL
//          v0_6_30:  2026-08-29  -KL
//          v0_6_31:  2026-08-29  -KL
//          v0_6_32:  2026-08-29  -KL
//          v0_6_33:  2026-08-30  -KL
//          v0_6_34:  2026-08-30  -KL
//          v0_6_35:  2026-08-30  -KL
//          v0_6_36:  2026-08-30  -KL
//          v0_6_37:  2026-08-31  -KL
//   ver. increment:  20260831--019 (v0_6_37)
//
//
//           author:  Kevin Lange
//      description:  Main code for Johnny 4 controller/transmitter
//                    running on a LILYGO TTGO T-Display v1.1 ESP32 board
//       update log:  v0_3 -- Changed potentiometer inputs to GPIOs on ADC1
//                    v0_4 -- Implemented ESP-NOW
//                    v0_5 -- Added ADS1115 ADC modules
//                    v0_6 -- Converted to sprite display
//                   v0_6b -- Migrated to PlatformIO; fixed keypad wiring
//                  v0_6_3 -- Comment cleanup and style normalization
//                  v0_6_4 -- Condensed; removed dead code; pot processing
//                            extracted into processPot() helper
//                  v0_6_5 -- Added UART link to XIAO ESP32S3 display board
//                  v0_6_6 -- Jukebox: receive file list chunks from j4_receiver,
//                            forward to j4_display via second UART packet type
//                  v0_6_7 -- Replaced String members in ESP-NOW structs with fixed
//                            char arrays (matching j4_receiver v0_7r_6)
//                         -- Control packet now carries need_filelist flag so the
//                            receiver re-sends the list if we boot late
//                         -- Answer LIST? requests from j4_display so a display
//                            reboot recovers the file list from our cached copy
//                  v0_6_8 -- Repurposed the eyes/spot/left-arm/right-arm pots into
//                            eye controls: iris (was eyes pot), eye-pop (was spot
//                            pot, 0-3200 like neck), eyes_x (was left-arm), eyes_y
//                            (was right-arm). ESP-NOW control packet now carries
//                            iris/eyes_x/eyes_y/eye_pop in place of eyes/spot/
//                            left_arm/right_arm. Neck joystick, volume, and the
//                            jukebox are unchanged. disp_pkt_t to j4_display keeps
//                            its layout (the new controls reuse the old slots).
//                  v0_6_9 -- Added a STATUS line to the TFT. The ESP-NOW status
//                            packet now carries a stepper_status field (from
//                            j4_stepper_neck via j4_receiver). Shows "ONLINE" when
//                            the link is up and all drivers are healthy, "OFFLINE"
//                            if no status packet arrives, or the reported fault
//                            (e.g. "NL OT", "EYES OFFLINE"). Green when healthy,
//                            red otherwise.
//                 v0_6_11 -- Added a screen-cycle button (TTGO built-in GPIO 35,
//                            like j4_receiver): data -> MAC address -> connection
//                            status. The connection screen shows ESP-NOW LINK,
//                            j4_stepper_neck, j4_stepper_eyes, j4_talk, and
//                            j4_display as CONNECTED/DISCONNECTED. The status packet
//                            carries neck_ok/eyes_ok/talk_ok from the receiver, the
//                            control packet carries display_ok, and j4_display +
//                            j4_talk send a "PING" heartbeat so they can be seen.
//                 v0_6_12 -- The stepper boards were consolidated: j4_stepper
//                            (renamed from j4_stepper_neck) now drives all five
//                            steppers and j4_stepper_eyes is retired. The status
//                            packet's neck_ok/eyes_ok became a single stepper_ok
//                            (matching j4_receiver v0_7r_12), and the connection
//                            screen lists j4_stepper instead of the two boards.
//                 v0_6_13 -- Second XIAO display board: j4_display_right (the old
//                            j4_display is renamed j4_display_left). It carries
//                            its own ADS1115 reading the four top-right pots and
//                            streams raw counts here over Serial2 (TX 25 / RX 26)
//                            as "P:<iris>,<color>,<brightness>,<volume>" at 25Hz.
//                            iris and volume now come from that feed (freeing
//                            ADS_01 A0/A1 for future pots), and two new controls
//                            ride the ESP-NOW control packet: color + brightness
//                            for the WS2812B strip on j4_receiver. display_ok
//                            split into display_l_ok / display_r_ok; the conn
//                            screen shows both displays.
//                 v0_6_14 -- The stepper boards split back into j4_stepper_neck +
//                            j4_stepper_eyes (per-endpoint limit switches needed
//                            the extra pins). Restored the eyes_ok field in the
//                            ESP-NOW status packet (matching j4_receiver
//                            v0_7r_14) and the j4_stepper_eyes row on the
//                            connection screen.
//                 v0_6_15 -- RF hardening for crowded venues (Open Sauce prep),
//                            matching j4_receiver v0_7r_15: ESP-NOW now runs on
//                            the ESP32 long-range (LR) PHY (~4dB more sensitive;
//                            ordinary WiFi gear cannot decode it -- both ends
//                            must be ESP32s in LR mode), TX power maxed at
//                            19.5dBm, and OnDataRecv drops any packet whose
//                            sender MAC is not our receiver. ESP-NOW channel
//                            pinned to 6 (ESPNOW_CHANNEL) to dodge other ESP-NOW
//                            projects on the default channel 1.
//                 v0_6_16 -- All control surfaces accounted for. Added ADS_03
//                            (0x4A) + ADS_04 (0x4B): the eight middle face pots
//                            (eyebrow L/R, basket eyebrow L/R, nose, nose basket,
//                            bottom eyelid L/R) now read on ADS_01 + ADS_03 and
//                            ride the ESP-NOW control packet to eight new PCA9685
//                            servo channels on j4_receiver (ch 6-13). ADS_04 is
//                            wired and initialized but its four channels are
//                            spares for the future pots (neck-pivot etc.).
//                            Added the four panel toggles on GPIO 32/33/13/15
//                            (LASER, VENT, EYE POP, AUX by the right joystick;
//                            INPUT_PULLUP, switch closes to GND). LASER + VENT
//                            drive PCA9685 ch 14/15 on j4_receiver; EYE POP
//                            replaces the old eye-pop pot (ADS_01 A2 freed) and
//                            sends 0 or 3200 through the existing stepper path;
//                            AUX is read + transmitted but unassigned. The
//                            control packet gains eight pot fields + a toggle
//                            bitmask (matching j4_receiver v0_7r_16).
//                 v0_6_17 -- Fixed a total lockup when any ADS1115 is absent:
//                            the ADS1X15 library's readADC() has no timeout,
//                            so with no chip on the bus isBusy() never clears
//                            and the first read in loop() spun forever (frozen
//                            screen, dead button, no ESP-NOW; yield() kept the
//                            watchdog fed so it never rebooted). Every module
//                            is now probed with isConnected() before its
//                            channels are read (adsReady()); absent modules
//                            read 0 (joystick axes centre at 1600) and are
//                            re-probed each pass, so the board runs fine bare
//                            on USB power and ADCs can even be hot-plugged.
//                 v0_6_18 -- Fixed the second lockup (~2s after boot, bare
//                            board): esp_now_send() ran every loop pass, only
//                            ever throttled by the old blocking ADC reads.
//                            With no ADCs the loop spun at multi-kHz, sends
//                            fired thousands of times a second, and
//                            OnDataSent (WiFi task) assigned to the
//                            connectStatus String each time -- heap ops
//                            racing loop()'s own, corrupting the heap in
//                            seconds. Control TX now runs on a 40ms timer
//                            (25 Hz, matching the receiver's 20ms gate), the
//                            callbacks only store plain flags, and the
//                            file-list forward to the display moved from
//                            OnDataRecv into loop() so Serial1 is written
//                            from one context only.
//                 v0_6_19 -- Fixed the ~2s-per-cycle stall on a bare board:
//                            with no I2C modules attached the bus has no
//                            pull-ups (they live on the breakouts), so a
//                            probe doesn't fast-NACK -- it eats the driver
//                            timeout + bus recovery, hundreds of ms each,
//                            and three probes per 40ms control cycle kept
//                            loop() almost always blocked (button worked
//                            only in the tiny gap between cycles).
//                            Wire.setTimeOut(10) caps every transaction,
//                            absent devices are re-probed only once per
//                            second (I2C_REPROBE_MS), and the keypad scan
//                            is gated on the PCF8574 ACKing (a floating bus
//                            read looks like a key held down forever).
//                 v0_6_20 -- Disconnect-audit residual: only "PING" / "LIST?"
//                            count as the j4_display_left heartbeat. Any line
//                            used to count, so garbage from the floating RX
//                            pin (display unplugged) faked CONNECTED.
//                 v0_6_21 -- Second 4x4 keypad. keypad_left is the NEW keypad
//                            (own PCF8574 backpack at 0x21 -- A0 jumper high;
//                            a PCF8574A backpack would land at 0x39 instead);
//                            keypad_right is the original at 0x20. Both drive
//                            the same phrase-select logic for now, scanned
//                            left-first, one key per pass, each with its own
//                            keymap string (keymap_left starts as a copy of
//                            keymap_right -- re-derive it if the new model's
//                            matrix comes out scrambled). Each pad has its
//                            own presence guard, so either can be absent or
//                            hot-plugged.
//                 v0_6_22 -- ADS_04 (0x4B) goes live: A0 is the neck-pivot
//                            pot (silver knob below j4_display_left, 0-3200
//                            like neck L/R, rides the control packet in the
//                            new neck_pivot field -> receiver -> nP on the
//                            stepper link), A1/A2 are two linear fader pots.
//                            fader_right doubles the IRIS pot (iris servo),
//                            fader_left doubles the Nose Basket pot (PCA9685
//                            ch 11). Each pair is arbitrated last-mover-wins:
//                            whichever control moved most recently is the
//                            active source and its value is used, so the two
//                            never fight. A source that is absent (module
//                            unplugged, display feed down) can neither claim
//                            nor hold active status.
//                 v0_6_23 -- FACE PRESETS. The right keypad stops doubling the
//                            phrase-select pad and becomes the face keypad:
//                            hold any key (except *) for 3s and j4_display_right
//                            asks "SAVE FACE ON <key>? PRESS * TO CONFIRM"
//                            (any other key cancels, 10s timeout). Confirming
//                            snapshots the current face (iris, color,
//                            brightness, the 8 face pots, and the LASER/VENT/
//                            EYE POP toggle states -- no volume, no neck, no
//                            eyes X/Y) into a 16-slot RAM table keyed by the
//                            keypad character, and pushes it through
//                            j4_receiver to j4_talk, which persists it in
//                            FACES.TXT on its microSD. Tapping a key recalls
//                            its face: a preset overlay holds every face
//                            channel at the recalled value until that
//                            channel's own physical control moves (frozen-
//                            baseline takeover, DUAL_CLAIM_COUNTS), and each
//                            toggle until its switch is flipped. On boot (or
//                            whenever the talk link appears) the controller
//                            re-requests the saved-face dump in the
//                            background until it has it; saves made while
//                            talk is offline stay in RAM flagged dirty and
//                            auto-sync when the link comes up. All of it is
//                            timer-driven ESP-NOW packet type 0x04 traffic --
//                            no board ever blocks or waits at boot for any
//                            of this. dualPick() baselines now freeze while
//                            a source is inactive so a slowly-moved control
//                            can still accumulate enough travel to claim.
//                            The controller now also talks TO j4_display_right
//                            (Serial2 TX GPIO 25, previously reserved):
//                            "M:<line1>|<line2>" shows a message, "X:" clears.
//                 v0_6_24 -- Screen-cycle button gains four more pages (still
//                            wraps: data -> MAC -> status -> ADS_01..ADS_04),
//                            showing each ADS1115 module's live raw pot
//                            counts and CONNECTED/DISCONNECTED for bench
//                            testing without a laptop on the I2C bus. Also
//                            corrected the README pin diagram: the physical
//                            header pin order was wrong on one rail (left
//                            and right rail assignment was fine, but pin
//                            order top-to-bottom on the left rail was never
//                            transformed for the 180-degree mounting
//                            rotation) and two GND pins were missing
//                            entirely (board is 12+12 pins, diagram only
//                            showed 11+11); verified against LilyGO's own
//                            pinout image, not just the schematic.
//                 v0_6_25 -- ADS_01 and ADS_02 swap payloads to match a
//                            rewire: both joysticks (eyes X/Y, neck X/Y)
//                            now land on ADS_01 (0x48) and the four
//                            left-bank face pots (Eyebrow L/R, Basket
//                            Eyebrow L/R) on ADS_02 (0x49). Addresses are
//                            unchanged -- ADS_01 is still 0x48 and ADS_02
//                            still 0x49, so no ADDR strap moves; only
//                            which pot wires land on which module's A0-A3.
//                 v0_6_26 -- ADS_01 channel order: neck joystick X/Y move
//                            to A0/A1 and eyes joystick X/Y to A2/A3
//                            (was the other way round). Scaling follows
//                            the signal, not the channel: neck/jaw stay
//                            0-3200, eyes stay 0-255.
//                 v0_6_27 -- ADS_03 A1 is faulty: the Nose Basket pot moves
//                            to ADS_04 A3 (previously spare) and A1 is left
//                            unread. Note this puts both halves of the Nose
//                            Basket dual-source pair (rotary pot + fader_left)
//                            on the same module, so dualPick() now takes
//                            ads4_ok for both sides and a missing ADS_04
//                            drops the pair together instead of leaving one
//                            source live.
//                 v0_6_28 -- fader_left now doubles the NECK-PIVOT pot
//                            (ADS_04 A0) instead of Nose Basket, so it is
//                            read on the 0-3200 neck scale rather than
//                            0-255. Nose Basket (ADS_04 A3) goes back to
//                            being a single-source rotary pot. The claim
//                            threshold had to become per-pair: 4 counts is
//                            ~1.5% of a 0-255 travel but only 0.125% of a
//                            0-3200 one, which sits inside pot noise and
//                            would have made the neck pair flip-flop
//                            between sources. dual_neck_pivot uses 50.
//                 v0_6_29 -- keypad_left fixed. Six of its keys (A, B, C,
//                            *, 0, #) were dead and the rest came out
//                            transposed. The keymap string was only half
//                            the problem: the scanner drove P0,P1,P2,P7
//                            and read P3,P4,P5,P6, which is right for the
//                            ORIGINAL pad but not the newer left one,
//                            whose 8 lines split straight down the middle
//                            (P0-P3 vs P4-P7). With the wrong split, any
//                            key whose two lines both sit in the drive set
//                            or both in the read set can never be seen at
//                            all -- exactly the 6 dead keys -- so no
//                            keymap edit could have recovered them. The
//                            drive/read split now travels with the pad
//                            (KeypadPins in KeypadGuard) and keymap_left
//                            was re-derived for it. Right pad unchanged.
//                 v0_6_30 -- Default (data) screen relabelled and rebuilt:
//                            Keypad_L / Playing / VOL / Keypad_R / Eye-X /
//                            Eye-Y / Neck-L / Neck-R / Neck-PIV, with IRIS
//                            and EYE-P dropped from the readout (both are
//                            still read and transmitted, just not shown).
//                            Neck-PIV surfaces the neck-pivot value, which
//                            had no on-screen readout before. The two pads
//                            also get their own last-key variables: both
//                            used to write last_key_char, so the single
//                            "Keypress" line showed whichever pad was
//                            touched last rather than a specific one.
//                            Update-log labels above were shifted +1 to
//                            match the renumbered header list.
//                 v0_6_31 -- Neck pivot and fader_left become centre-zero:
//                            -1600 at minimum, 0 at the midpoint, +1600 at
//                            maximum, with fader_left inverted so it and
//                            the pot drive the pivot the same direction.
//                            processPotCentred() adds the jitter filtering
//                            a stepper axis needs: a deadband around centre
//                            (rest = exactly 0, a hard stop, not a small
//                            standing offset) and hysteresis elsewhere in
//                            travel (noise below POT_HYSTERESIS cannot
//                            re-issue a move). Each control keeps its own
//                            filter state.
//                            The ESP-NOW field stays 0-3200 absolute and is
//                            re-centred at transmit: j4_stepper_neck turns
//                            nP into an ABSOLUTE position (nP/2) homed off
//                            the MIN limit switch, so signed values on the
//                            wire would command negative positions and run
//                            the pivot into that switch. Keeping the wire
//                            contract also means the receiver and stepper
//                            need no reflash to match this build.
//                            dualPick() lost its -1 "not seen yet" sentinel
//                            for explicit seen flags -- with a centre-zero
//                            pair, -1 is a real value just left of centre,
//                            and the sentinel made the pot unable to claim
//                            back from the fader anywhere in negative travel.
//                 v0_6_32 -- Anti-jitter for the neck joystick axes, which
//                            matters now the springs are out and the stick
//                            can sit parked off-centre: a couple of counts
//                            of ADC noise per axis reached the mixer as a
//                            couple of counts on neck-L and neck-R, i.e.
//                            constant physical stepper motion on a joystick
//                            nobody was touching.
//                            Filtered at the two axis reads, before the
//                            mixer -- filtering the mixer outputs instead
//                            would leave each axis free to jitter into the
//                            other.
//                            New stickyBand() replaces the report-on-change
//                            threshold everywhere, including inside
//                            processPotCentred(). The old form quantised
//                            real movement into threshold-sized hops, so a
//                            slow move counted 10 at a time instead of 1.
//                            The sticky band instead lets the reported value
//                            TRAIL the live one by up to `band`, dragged
//                            rather than snapped: dead still at rest, but
//                            full 1-count resolution while moving, at the
//                            cost of `band` counts of lag. That keeps the
//                            3200-count precision on the neck axes.
//                            Filter states are globals now, and the read
//                            block's module-absent branches reset them so a
//                            returning module is not dragged out of a stale
//                            value.
//                 v0_6_33 -- Fader windows, eye joystick recentring, spike
//                            rejection, NECK PIV pot disabled.
//                            Both faders now use only the bottom 40% of their
//                            mechanical travel: fader_left reads -2048 at 0%,
//                            0 at 20%, +2048 at 40%; fader_right 0 / 127 / 255
//                            across the same window. Past 40% each pins at its
//                            high end. The window is defined by MEASURED raw
//                            endpoints (FADER_*_RAW_BOTTOM/_TOP), not by
//                            percentages of the ADC range, because a fader's
//                            electrical span reaches neither the ADC rails nor
//                            the ends of its own mechanical travel -- which is
//                            what left dead motion at the bottom of fader_left.
//                            THE SHIPPED ENDPOINTS ARE PLACEHOLDERS and must be
//                            calibrated on the bench; the ADS screens now show
//                            RAW counts so they can be read off directly.
//                            NECK PIV pot (ADS_04 A0) is disabled as a control:
//                            still read so its raw shows on screen for whatever
//                            it gets repurposed to, but the neck pivot is driven
//                            by fader_left alone, so that dual-source pair and
//                            its claim threshold are gone. Iris is the only
//                            arbitrated pair left.
//                            Eye joystick is centre-zero -128..0..128 on both
//                            axes, mapped per-half from the six measured
//                            calibration points: its electrical centre is not
//                            the midpoint of its travel and the halves differ
//                            in width, so one straight map would put neutral
//                            off-zero and reach one extreme early. Neutral now
//                            transmits 128 = true servo centre (it used to send
//                            the stick's 135). Small deadzone at neutral.
//                            median3() rejects dirty-wiper spikes on the raw
//                            counts of every motion control. A median drops any
//                            single-sample outlier however far out it lands,
//                            where averaging or slew-limiting would smear every
//                            fast move to soften the rare bad one. Costs one
//                            sample (40ms) of lag and nothing else.
//                            Wire formats are all unchanged -- eyes still go out
//                            0-255 and nP still 0-3200 absolute, rescaled at
//                            transmit -- so no other board needs reflashing.
//                            The display packet's eyes slots are uint8_t, so
//                            they get the same re-centring; casting the signed
//                            value straight in would have wrapped it.
//                 v0_6_34 -- Bench corrections to v0_6_33.
//                            Eyes read exactly 0/0 at rest again. The deadzone
//                            was not the problem: it did map neutral to 0, but
//                            stickyBand() trails its input by up to `band` and
//                            so parked one count off and stayed there, giving
//                            the -1 / +1 at rest. New eyeAxis() forces a hard 0
//                            inside the deadzone instead of dragging toward it,
//                            the same way processPotCentred() already did for
//                            the centre-zero pots. Deadzone also widened 6->10
//                            as asked. Verified 0 non-zero reads in 2000 from
//                            any prior state.
//                            fader_left calibrated: measured 17560 raw at its
//                            physical bottom, so FADER_L_RAW_BOTTOM was 160
//                            counts too low and pinned the output at -2048
//                            across that span (the pivot not responding until
//                            ~17403). 17560 counts is 3.29V, so the faders do
//                            span the full rail and the 40% endpoints are that
//                            span scaled. fader_right measured 0 at its bottom,
//                            as assumed.
//                            ADS screens now show eight values per module
//                            instead of four: each channel's raw count, and
//                            under it what that channel drives plus the
//                            processed value that leaves this board. ADS_01 A1
//                            relabelled JAW Y -> NECK Y.
//                 v0_6_35 -- Neck-L and Neck-R become centre-zero too
//                            (-1600..0..1600, matching the neck pivot), and
//                            the pivot's own range narrows from +/-2048 to
//                            +/-1600. Both joystick axes are offset to
//                            centre-zero BEFORE the differential mix rather
//                            than after: mixing in the old 0-3200 domain and
//                            subtracting afterwards would clamp against the
//                            wrong ends of the range.
//                            Wire formats unchanged again -- nL/nR/nP all
//                            still go out 0-3200 absolute, re-centred at
//                            transmit, because j4_stepper_neck turns each
//                            into an absolute position homed off that axis's
//                            MIN limit switch. The display packet's neck
//                            slots are uint8_t and now map from the signed
//                            range; left mapping from 0-3200 every negative
//                            value would have wrapped to a large byte, the
//                            same trap the eyes slots hit in v0_6_33.
//                            Eye deadzone back to 6 from 10.
//                 v0_6_36 -- fader_left's window narrowed and recentred:
//                            -1600 / 0 / +1600 now fall on raw 17560 / 15000
//                            / 12440. The halves are equal (2560 counts each)
//                            so centre lands exactly on 15000.
//                            That window is 5120 raw counts against the old
//                            7024, so the fader is now more sensitive: about
//                            1.6 raw counts per output count where a neck
//                            axis gets 5.3. The shared POT_STICKY_BAND was
//                            sized for the wider window, so fader_left gets
//                            its own FADER_L_STICKY_BAND, set to 7 by
//                            simulation: the value stops moving at rest from
//                            a band of 5 up, and 7 leaves margin while still
//                            costing only 0.22% of travel in lag. The old
//                            wider window was already marginal at the shared
//                            band of 3 (440 changes per 3000 reads at rest),
//                            so this fixes a latent problem rather than one
//                            the narrowing introduced.
//                 v0_6_37 -- Eighth screen: the fifth ADS1115 in the system,
//                            the one on j4_display_right (IRIS, COLOR,
//                            BRIGHTNESS, VOLUME). It is not on this board's
//                            I2C bus, so its raw counts come from the "P:"
//                            feed over Serial2 and CONNECTED means that feed
//                            is fresh rather than that a chip ACKed. A stale
//                            feed blanks the counts to "--" instead of
//                            leaving the last values frozen and looking live.
//                            IRIS shows the arbitrated value, so it can
//                            disagree with this pot's own raw while
//                            fader_right is the active source -- that
//                            disagreement is the quickest read on which
//                            source currently owns the channel.
//
//
//
//      J4_CONTROLLER (RC Controller / Transmitter for Johnny 4 project)
//      ------------------------------------------------------------------
//      LilyGO TTGO T-Display v1.1 (ESP32) Module's Pin Connections
//      ------------------------------------------------------------------
//      VIN:
//      GND:  Make sure all grounds are connected together
//     3.3V:
//        0:  button 1 / BOOT      [USED BY TTGO]
//        4:  TFT backlight        [USED BY TTGO]
//        5:  TFT CS               [USED BY TTGO]
//       16:  TFT DC               [USED BY TTGO]
//       17:  DISPLAY-L TX  →  left XIAO D7 / GPIO44   (Serial1)
//       18:  TFT SCLK             [USED BY TTGO]
//       19:  TFT MOSI             [USED BY TTGO]
//
//       13:  EYE POP toggle (INPUT_PULLUP, switch closes to GND)
//       15:  AUX toggle, next to the right joystick (INPUT_PULLUP, unassigned)
//
//       21:  SDA  [I2C BUS] (ADS_01 0x48, ADS_02 0x49, ADS_03 0x4A,
//                            ADS_04 0x4B, keypad 0x20)
//       22:  SCL  [I2C BUS]
//
//       23:  TFT RST              [USED BY TTGO]
//
//       25:  DISPLAY-R TX  →  right XIAO D7 / GPIO44  (Serial2, face messages)
//       26:  DISPLAY-R RX  ←  right XIAO D6 / GPIO43  (Serial2, pot feed)
//       27:  DISPLAY-L RX  ←  left XIAO D6 / GPIO43   (Serial1)
//
//       32:  LASER toggle (INPUT_PULLUP, switch closes to GND)
//       33:  VENT  toggle (INPUT_PULLUP, switch closes to GND)
//
//       34:  battery voltage sense
//
//       35:  button 2             [USED BY TTGO]
//      ------------------------------------------------------------------
//      ------------------------------------------------------------------
//
//      XIAO LINK WIRING  (3.3V logic on both sides - no level shifter needed)
//      ------------------------------------------------------------------
//      j4_display_left  (Serial1):
//        TTGO GPIO17  →  XIAO D7 (GPIO44)    TTGO TX → XIAO RX
//        TTGO GPIO27  ←  XIAO D6 (GPIO43)    TTGO RX ← XIAO TX
//      j4_display_right (Serial2):
//        TTGO GPIO25  →  XIAO D7 (GPIO44)    TTGO TX → XIAO RX (face messages)
//        TTGO GPIO26  ←  XIAO D6 (GPIO43)    TTGO RX ← XIAO TX (pot feed)
//      TTGO GND  -  both XIAO GNDs
//      ------------------------------------------------------------------
//
//      ANALOG INPUTS
//      ------------------------------------------------------------------
//      Local, on the four ADS1115 ADCs (I2C):
//      ADS_01 (0x48) A0:  neck joystick X
//      ADS_01 (0x48) A1:  neck joystick Y (jaw)  -> mixed into neck-L / neck-R
//      ADS_01 (0x48) A2:  eyes joystick X / eyes_x        -> eyes pan servo
//      ADS_01 (0x48) A3:  eyes joystick Y / eyes_y        -> eyes tilt servo
//      ADS_02 (0x49) A0:  Eyebrow L pot         -> PCA9685 ch 6
//      ADS_02 (0x49) A1:  Eyebrow R pot         -> PCA9685 ch 7
//      ADS_02 (0x49) A2:  Basket Eyebrow L pot  -> PCA9685 ch 8
//      ADS_02 (0x49) A3:  Basket Eyebrow R pot  -> PCA9685 ch 9
//      ADS_03 (0x4A) A0:  Nose pot (up/down)    -> PCA9685 ch 10
//      ADS_03 (0x4A) A1:  FAULTY -- unused, left unread (Nose Basket moved
//                         off this channel to ADS_04 A3 on 2026-08-29)
//      ADS_03 (0x4A) A2:  Bottom Eyelid L pot   -> PCA9685 ch 12
//      ADS_03 (0x4A) A3:  Bottom Eyelid R pot   -> PCA9685 ch 13
//      ADS_04 (0x4B) A0:  neck-pivot pot (silver knob below j4_display_left)
//                         -> nP on the stepper link. Read centre-zero
//                         (-1600..0..1600), sent as 0-3200 absolute.
//      ADS_04 (0x4B) A1:  fader_left  (linear fader) -> neck pivot, shared
//                         with ADS_04 A0 (last-mover-wins). Same centre-zero
//                         scale, read INVERTED so both move the pivot alike.
//      ADS_04 (0x4B) A2:  fader_right (linear fader) -> iris, shared with
//                         j4_display_right's IRIS pot (last-mover-wins)
//      ADS_04 (0x4B) A3:  Nose Basket pot       -> PCA9685 ch 11
//                         (single source; fader_left no longer doubles it)
//
//      TOGGLE INPUTS (INPUT_PULLUP, switch closes to GND, ON = LOW):
//      GPIO 32:  LASER   -> PCA9685 ch 14 servo on j4_receiver
//      GPIO 33:  VENT    -> PCA9685 ch 15 servo on j4_receiver
//      GPIO 13:  EYE POP -> eye-pop steppers, sends 0 (normal) or 3200 (popped)
//      GPIO 15:  AUX (next to the right joystick, unassigned; sent as a spare bit)
//
//      Remote, from j4_display_right's ADS1115 (0x48 on its own bus) over
//      Serial2 ("P:" lines):
//        iris        -> iris servo (PCA9685 on j4_receiver)
//        color       -> WS2812B strip color   (j4_receiver)
//        brightness  -> WS2812B strip brightness (j4_receiver)
//        volume      -> j4_talk audio volume
//
//      Everything is sent to j4_receiver over ESP-NOW. The receiver drives
//      the face/eye servos + LED strip, forwards neck + eye-pop to
//      j4_stepper_neck, and relays volume to j4_talk.
//      ------------------------------------------------------------------
//
//
//
//******************************************************************************


#include <Wire.h>
#include <PCF8574.h>
#include <TFT_eSPI.h>
#include <ADS1X15.h>
#include <SPI.h>
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "kevco_labs_logo_02.h" // 135 x 37 pixels

// ----------  FUNCTION PROTOTYPES ----------
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len);
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
void labelsDisplaySprite();
void dataDisplaySprite();
void tftDisplayUpdate();
void controllerScreenModeDetect();
void macAddressDisplay();
void connectionDisplay();
void drawConnLine(const char *name, bool ok, int row);
void adsPotsDisplay(int idx);
void sendToXIAO();
void sendFileListToXIAO();
// ------------------------------------------


#define SDA 21
#define SCL 22

// --- XIAO LINKS ---
#define XIAO_TX_PIN  17    // Serial1: GPIO17 output → left XIAO D7 (GPIO44)
#define XIAO_RX_PIN  27    // Serial1: GPIO27 input  ← left XIAO D6 (GPIO43)
#define XIAOR_TX_PIN 25    // Serial2: GPIO25 output → right XIAO D7 (reserved)
#define XIAOR_RX_PIN 26    // Serial2: GPIO26 input  ← right XIAO D6 (pot feed)
#define XIAO_BAUD    115200
#define JOYSTICK_DEAD_ZONE 200  // counts either side of 1600 to snap to center (scaled for 0-3200 range)

// Panel toggle switches (INPUT_PULLUP, switch closes to GND, ON = LOW)
#define LASER_TOGGLE_PIN    32  // -> laser servo, PCA9685 ch 14 on j4_receiver
#define VENT_TOGGLE_PIN     33  // -> vent servo,  PCA9685 ch 15 on j4_receiver
#define EYE_POP_TOGGLE_PIN  13  // -> eye-pop steppers, 0 (normal) / 3200 (popped)
#define AUX_TOGGLE_PIN      15  // next to the right joystick; unassigned spare

// Toggle bit positions in xmitData.toggles_xmit (1 = switch ON)
#define TOGGLE_BIT_LASER    0
#define TOGGLE_BIT_VENT     1
#define TOGGLE_BIT_EYE_POP  2
#define TOGGLE_BIT_AUX      3

// Binary packet sent to the XIAO display board over Serial1.
// Both ends must keep this struct identical.
typedef struct __attribute__((packed)) {
  uint8_t  magic[2];       // 0xAB, 0xCD - frame sync marker
  uint8_t  volume;         // 0-100
  uint8_t  eyes;           // 0-255
  uint8_t  spot;           // 0-255
  uint8_t  left_arm;       // 0-255
  uint8_t  right_arm;      // 0-255
  uint8_t  neck;           // 0-255
  uint8_t  jaw;            // 0-255
  uint16_t bat1_mv;        // controller battery in millivolts
  int16_t  bat2_raw;       // robot battery 2 - raw value from receiver
  int16_t  bat3_raw;       // robot battery 3 - raw value from receiver
  uint8_t  connect_ok;     // 1 = ESP-NOW link healthy, 0 = error
  char     phrase[32];     // null-terminated phrase name (e.g. "A12")
  uint8_t  checksum;       // XOR of all preceding bytes in the packet
} disp_pkt_t;

// File list packet - second packet type on the same UART link.
// Magic 0xBE, 0xCD distinguishes it from the control packet (0xAB, 0xCD).
// Sent once after the file list is fully received from j4_receiver.
#define MAX_FILES      50
#define FILE_ID_LEN     2
#define FILE_NAME_MAX  22
#define FILES_PER_CHUNK 5

typedef struct __attribute__((packed)) {
  uint8_t magic[2];        // 0xBE, 0xCD
  uint8_t total_files;
  struct {
    char id[FILE_ID_LEN];
    char name[FILE_NAME_MAX];
  } files[MAX_FILES];
  uint8_t checksum;
} filelist_pkt_t;

// ESP-NOW packet type byte - first byte of every payload
#define ESPNOW_PKT_CONTROL  0x01  // controller -> receiver (sticks, pots, phrase select)
#define ESPNOW_PKT_FILELIST 0x02  // receiver -> controller (file list chunk)
#define ESPNOW_PKT_STATUS   0x03  // receiver -> controller (now playing, batteries)
#define ESPNOW_PKT_FACE     0x04  // face presets, both directions (op says which)

// Face packet ops (espnow_face_pkt_t.op). The receiver translates these
// to/from text lines on the Teensy UART; this struct MUST stay byte-identical
// with j4_receiver's copy.
#define FACE_OP_SAVE 1  // controller -> receiver: write this face to the SD
#define FACE_OP_REQ  2  // controller -> receiver: send me the saved-face dump
#define FACE_OP_DATA 3  // receiver -> controller: one saved face from the dump
#define FACE_OP_END  4  // receiver -> controller: dump complete, v[0] = count
#define FACE_OP_ACK  5  // receiver -> controller: Teensy wrote the slot to SD
#define FACE_OP_ERR  6  // receiver -> controller: Teensy SD write failed

#define FACE_VALUES 11  // iris, color, brightness, the 8 face pots (FI_* order)

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;          // ESPNOW_PKT_FACE
  uint8_t op;                // FACE_OP_*
  uint8_t key;               // keypad character ('0'-'9','A'-'D','#'); 0 = unused
  uint8_t toggles;           // bit 0 LASER, 1 VENT, 2 EYE POP
  int16_t v[FACE_VALUES];
} espnow_face_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint8_t chunk_index;
  uint8_t total_chunks;
  uint8_t entry_count;
  struct {
    char id[FILE_ID_LEN];
    char name[FILE_NAME_MAX];
  } entries[FILES_PER_CHUNK];
} espnow_filelist_chunk_t;
// --- END XIAO LINK ---


ADS1115 ADS_01(0x48);  // ADDRESS PIN TO GND
ADS1115 ADS_02(0x49);  // ADDRESS PIN TO VDD
ADS1115 ADS_03(0x4A);  // ADDRESS PIN TO SDA
ADS1115 ADS_04(0x4B);  // ADDRESS PIN TO SCL (A0 neck-pivot, A1/A2 faders, A3 spare)


// --- ESP-NOW RELATED ---
uint8_t broadcastAddress[] = { 0xA0, 0xDD, 0x6C, 0x74, 0xDA, 0x74 };  //MAY 2026 TTGO 2026-05-01--1238-KL

// ESP-NOW WiFi channel -- MUST match j4_receiver. 6 avoids the ESP32 power-on
// default (1) that every unconfigured ESP-NOW project sits on. Use 1/6/11 only.
#define ESPNOW_CHANNEL 6

// Both ends must keep these structs identical to the ones in j4_receiver.
// Packed with fixed-size char arrays -- no String members, they don't survive
// the memcpy across ESP-NOW (the receiver would get a pointer, not the text).
typedef struct __attribute__((packed)) struct_message_rcv {
  uint8_t pkt_type;                  // ESPNOW_PKT_STATUS
  char    phrase_playing_rcv[32];
  int16_t battery_02_voltage_rcv;
  int16_t battery_03_voltage_rcv;
  char    stepper_status_rcv[24];    // from j4_stepper_neck via j4_receiver
  uint8_t stepper_ok_rcv;            // 1 = j4_stepper_neck responding
  uint8_t eyes_ok_rcv;               // 1 = j4_stepper_eyes responding (per neck's EY:)
  uint8_t talk_ok_rcv;               // 1 = j4_talk (Teensy) responding
} struct_message_rcv;

struct_message_rcv rcvData;

typedef struct __attribute__((packed)) struct_message_xmit {
  uint8_t pkt_type;                  // ESPNOW_PKT_CONTROL
  char    phrase_select_xmit[8];
  int16_t volume_xmit;
  int16_t iris_xmit;                 // 270-deg iris servo (pot on j4_display_right)
  int16_t color_xmit;                // WS2812B strip color, 0-255 (j4_display_right)
  int16_t brightness_xmit;           // WS2812B strip brightness, 0-255 (j4_display_right)
  int16_t eyes_x_xmit;               // eyes joystick X -> eyes_x servo
  int16_t eyes_y_xmit;               // eyes joystick Y -> eyes_y servo
  int16_t eye_pop_xmit;              // eye-pop steppers, 0 or 3200 (EYE POP toggle)
  int16_t neck_left_xmit;
  int16_t neck_right_xmit;
  int16_t neck_pivot_xmit;           // nP on the stepper link, 0-3200 absolute
                                     // (re-centred from the internal -1600..1600)
  int16_t eyebrow_l_xmit;            // Eyebrow L pot        -> PCA9685 ch 6
  int16_t eyebrow_r_xmit;            // Eyebrow R pot        -> PCA9685 ch 7
  int16_t basket_brow_l_xmit;        // Basket Eyebrow L pot -> PCA9685 ch 8
  int16_t basket_brow_r_xmit;        // Basket Eyebrow R pot -> PCA9685 ch 9
  int16_t nose_xmit;                 // Nose pot (up/down)   -> PCA9685 ch 10
  int16_t nose_basket_xmit;          // Nose Basket pot      -> PCA9685 ch 11
  int16_t eyelid_l_xmit;             // Bottom Eyelid L pot  -> PCA9685 ch 12
  int16_t eyelid_r_xmit;             // Bottom Eyelid R pot  -> PCA9685 ch 13
  uint8_t toggles_xmit;              // bit 0 LASER, 1 VENT, 2 EYE POP, 3 AUX
  uint8_t need_filelist_xmit;        // 1 = still waiting on the file list
  uint8_t display_l_ok_xmit;         // 1 = controller sees j4_display_left heartbeat
  uint8_t display_r_ok_xmit;         // 1 = controller sees j4_display_right pot feed
} struct_message_xmit;

struct_message_xmit xmitData;
esp_now_peer_info_t peerInfo;

volatile bool connectError = LOW;
String connectStatus = "NO INFO";

// Status line: tracks the last status packet from j4_receiver. If none arrives
// within the timeout, the ESP-NOW link is treated as down ("OFFLINE").
unsigned long lastStatusRecvMs = 0;
#define STATUS_LINK_TIMEOUT_MS  1500

// j4_display_left heartbeat ("PING" or any line on the Serial1 link)
unsigned long lastDisplayMs = 0;
// j4_display_right heartbeat (its 25Hz "P:" pot lines on the Serial2 link)
unsigned long lastDisplayRMs = 0;
#define DISPLAY_TIMEOUT_MS  3000

// Screen cycling via the TTGO's built-in button on GPIO 35 (same as j4_receiver).
// 0 = data, 1 = MAC address, 2 = connection status, 3-7 = live ADS1115 pot
// counts (one screen per module: ADS_01 through ADS_04, then the fifth
// ADS1115 in the system, the one on j4_display_right, via its "P:" feed).
#define SCREEN_BUTTON  35
#define NUM_SCREENS    8
int  screen_mode = 0;
bool screen_button_prev = HIGH;
unsigned long screen_button_previousMillis = 0;
const unsigned long screen_debounce_ms = 50;
// --- END ESP-NOW RELATED ---

// --- JUKEBOX FILE LIST ---
filelist_pkt_t jukeboxPkt;
bool           jukeboxReady       = false;
uint8_t        chunksReceived     = 0;
uint8_t        chunksExpected     = 0;
String         xiaoSerialBuf      = "";
// --- END JUKEBOX FILE LIST ---


// Potentiometer values
int volume_value    = 0;   // from j4_display_right (was ADS_01 A0)
int iris_value      = 0;   // from j4_display_right (was ADS_01 A1)
int color_value     = 0;   // from j4_display_right -> WS2812B color
int brightness_value = 0;  // from j4_display_right -> WS2812B brightness
int eyes_x_value    = 0;   // eyes joystick X (was left-arm pot)
int eyes_y_value    = 0;   // eyes joystick Y (was right-arm pot)
int eye_pop_value   = 0;   // eye-pop steppers, 0 or 3200 (EYE POP toggle)
int neck_value       = 0;  // joystick X-axis raw
int jaw_value        = 0;  // joystick Y-axis raw
int neck_left_value  = 0;
int neck_right_value = 0;

// stickyBand() state, one per filtered control. Globals rather than statics
// inside the read block so the read block's "module absent" branch can reset
// them: a returning module must not be dragged out of a stale old value.
// The neck axes are filtered because with the joystick springs removed it can
// be parked off-centre, where a couple of counts of ADC noise on each axis
// reaches the mixer as a couple of counts on neck-L and neck-R -- constant
// physical stepper motion on a joystick nobody is touching.
int neck_sticky = 1600, jaw_sticky = 1600;   // 0-3200 axes, centre 1600
int fl_sticky   = 0;                         // fader_left, centre-zero
int eyex_sticky = 0,    eyey_sticky = 0;     // eye joystick, centre-zero

// (median3() filter state lives with median3() itself, further down --
//  it needs the struct definition, which sits with the other input filters)

// Last raw ADS1115 counts, [module][channel], -1 = not read this pass.
// Modules 0-3 are this board's own; module 4 is the fifth ADS1115 in the
// system, the one on j4_display_right, whose counts arrive over Serial2.
// Shown on the per-module screens: raw is what you need to calibrate a fader
// window or a joystick centre, and it is the only place raw is visible.
int ads_raw[5][4] = {
  { -1, -1, -1, -1 }, { -1, -1, -1, -1 }, { -1, -1, -1, -1 },
  { -1, -1, -1, -1 }, { -1, -1, -1, -1 }
};
int neck_pivot_value = 0;     // neck-pivot pot (ADS_04 A0), -1600..0..1600; 0 = centre/stop

// Linear fader pots (ADS_04 A1/A2). Each doubles an existing rotary pot:
// fader_left pairs with the neck-pivot pot (ADS_04 A0) and so shares its
// centre-zero -1600..0..1600 scale (and is read inverted, so both controls
// push the pivot the same direction); fader_right with the IRIS pot on
// j4_display_right, still 0-255. See dualPick() for the arbitration.
int fader_left_value  = 0;      // centre-zero: 0 = centre/stop
int fader_right_value = 0;

// Middle face pots (0-255, mapped to PCA9685 servo channels on j4_receiver)
int eyebrow_l_value     = 0;  // ADS_02 A0
int eyebrow_r_value     = 0;  // ADS_02 A1
int basket_brow_l_value = 0;  // ADS_02 A2
int basket_brow_r_value = 0;  // ADS_02 A3
int nose_value          = 0;  // ADS_03 A0
int nose_basket_value   = 0;  // ADS_04 A3
int eyelid_l_value      = 0;  // ADS_03 A2
int eyelid_r_value      = 0;  // ADS_03 A3

// Panel toggles (true = switch ON, pin pulled to GND)
bool laser_toggle   = false;
bool vent_toggle    = false;
bool eye_pop_toggle = false;
bool aux_toggle     = false;

// Raw ADS1115 counts streamed from j4_display_right's "P:" lines. Held raw so
// the same processPot() scaling applies as for the local ADS channels.
int dispR_iris_raw       = 0;
int dispR_color_raw      = 0;
int dispR_brightness_raw = 0;
int dispR_volume_raw     = 0;
String xiaoRSerialBuf    = "";

// Controller battery (millivolts), updated by battery timer, read by sendToXIAO()
uint16_t bat1_mv = 0;


// --- FACE PRESETS ---
// A face is the 11 pot channels below plus the LASER/VENT/EYE POP toggle
// states. No volume, no neck, no eyes X/Y. Slots are keyed by the keypad
// CHARACTER (not the scan code) so a re-derived keymap keeps every saved
// face on the same printed key. '*' is the confirm key and cannot be a slot.
enum {
  FI_IRIS = 0, FI_COLOR, FI_BRIGHT,
  FI_BROW_L, FI_BROW_R, FI_BBROW_L, FI_BBROW_R,
  FI_NOSE, FI_NOSE_BASKET, FI_LID_L, FI_LID_R
};

#define FACE_SLOTS 16
struct FaceSlot {
  char    key;         // keypad character, 0 = slot never used
  bool    valid;       // has data (recallable)
  bool    dirty;       // not yet confirmed on the Teensy SD
  bool    sd_failed;   // Teensy reported a write error -- stop auto-retrying
  int16_t v[FACE_VALUES];
  uint8_t toggles;     // bit 0 LASER, 1 VENT, 2 EYE POP
};
FaceSlot faces[FACE_SLOTS];

bool          faces_synced   = false;  // true once a complete SD dump landed
uint8_t       faceDumpCount  = 0;      // FACE_OP_DATA packets since last REQ
unsigned long faceReq_previousMillis   = 0;
unsigned long faceDirty_previousMillis = 0;
const unsigned long faceReq_interval   = 2500;  // re-request dump until synced
const unsigned long faceDirty_interval = 2000;  // push one dirty face per tick

// Preset overlay: a recalled face holds each channel until that channel's own
// physical control moves (same frozen-baseline takeover as dualPick), and
// each toggle until its physical switch is flipped.
int16_t preset_v[FACE_VALUES];
bool    preset_on[FACE_VALUES] = { false };
uint8_t preset_toggles     = 0;   // recalled toggle states
uint8_t preset_toggle_mask = 0;   // bit set = that toggle still preset-driven
int16_t phys_baseline[FACE_VALUES];
bool    phys_baseline_init = false;
uint8_t toggles_phys_last  = 0;

// Right keypad (face keypad) press/hold tracking + save-confirm prompt
#define FACE_HOLD_MS            3000   // hold this long to open the save prompt
#define FACE_PROMPT_TIMEOUT_MS 10000   // unanswered prompt cancels itself
int8_t        rk_down       = -1;     // scan code currently held, -1 = none
unsigned long rk_down_since = 0;
bool          rk_hold_fired = false;  // this press already opened the prompt
bool          rk_consumed   = false;  // this press answered the prompt
char          face_prompt_key     = 0;   // 0 = no prompt showing
unsigned long face_prompt_started = 0;
unsigned long faceMsg_clearAt     = 0;   // 0 = message is not timed

// Incoming face packets: OnDataRecv (WiFi task) only memcpys into this ring;
// loop() drains it. Single producer / single consumer, volatile indexes.
#define FACE_RXQ 8
espnow_face_pkt_t faceRxQ[FACE_RXQ];
volatile uint8_t  faceRxHead = 0;
volatile uint8_t  faceRxTail = 0;
// --- END FACE PRESETS ---


// Timers
unsigned long tft_update_previousMillis = 0;
unsigned long battery_01_previousMillis = 0;
unsigned long keypad_previousMillis     = 0;
unsigned long control_tx_previousMillis = 0;
const unsigned long tft_update_interval = 40;   // 25 fps
const unsigned long battery_01_interval = 500;
const unsigned long keypad_interval     = 150;
const unsigned long control_tx_interval = 40;   // 25 Hz control packets

// Set by OnDataRecv (WiFi task), consumed by loop(): forward the completed
// jukebox file list to j4_display_left from loop context only.
volatile bool filelistForwardPending = false;


// Two 4x4 matrix keypads, each on its own PCF8574 I2C backpack:
//   keypad_left  -- the NEW keypad, left side of the panel. ADDR jumpered to
//                   0x21 (A0 high). If its backpack is a PCF8574A, the same
//                   jumper lands at 0x39 instead -- change the two 0x21s.
//   keypad_right -- the original keypad at 0x20 (all jumpers low).
// For now both drive exactly the same phrase-select logic.
PCF8574 pcf_left(0x21);
PCF8574 pcf_right(0x20);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite screen_bottom_sprite_203 = TFT_eSprite(&tft);

// Keypad wiring: keypad pin 1 plugs into P0 straight through on both pads,
// but the two pads are different models and do NOT put rows and columns on
// the same pins, which is why the drive/read split is per-pad (KeypadPins).
//   RIGHT (original): P0=Row3, P1=Row2, P2=Row1, P3=Col4, P4=Col3, P5=Col2,
//                     P6=Col1, P7=Row4  -- rows land on P0,P1,P2,P7, so P3
//                     and P7 are swapped vs the I2CKeyPad library's
//                     expectation and we scan manually.
//   LEFT  (newer):    one axis on P0-P3, the other on P4-P7, a straight 4+4
//                     split (see KP_PINS_LEFT).
// Keymaps indexed by (drive_index*4 + read_index) using that pad's own
// KeypadPins order. N = NoKey, F = Fail.
//
// keymap_right was derived on the bench for the original keypad.
// keymap_left was derived on the bench 2026-08-29 for the newer pad, whose
// lines land on the PCF8574 as:
//   P0-P3 = the {1,4,7,*} {2,5,8,0} {3,6,9,#} {A,B,C,D} groups (driven)
//   P4-P7 = the {1,2,3,A} {4,5,6,B} {7,8,9,C} {*,0,#,D} groups (read)
// so index = drive*4 + read walks the printed grid one column at a time.
char keymap_left[19]  = "147*2580369#ABCDNF";
char keymap_right[19] = "#9630852*741DCBANF";
// Last decoded key from each pad, shown on the data screen as Keypad_L /
// Keypad_R. Kept separate: before v0_6_30 both pads wrote last_key_char, so
// the single "Keypress" line showed whichever pad was touched most recently.
char last_key_char       = 'N';   // left pad  (phrase select)
char last_key_char_right = 'N';   // right pad (face presets)
int key      = -2;
int old_key  = -1;
String phrase_select_buffer = "";
bool ready_message = true;


// Drive one axis LOW a line at a time, check whether any line of the other
// axis reads LOW. Which PCF8574 pins carry which axis is PER KEYPAD -- the
// two pads are different models and split their 8 lines at different pins,
// so the drive/read sets travel with the keypad, not baked in globally.
//
// A matrix key is only detectable if one of its two lines is in the drive
// set and the other is in the read set. Get the split wrong and the keys
// whose lines both land in the same set are invisible no matter what the
// keymap says -- that is the failure the left pad showed on the bench
// (its 6 dead keys), not a scrambled keymap.
struct KeypadPins {
  uint8_t driveWrite[4];   // ~(1 << pin), one drive line pulled LOW
  uint8_t readBit[4];      // pin numbers of the read lines
  uint8_t readMask;        // bit mask of the read lines (all read pins HIGH)
};

// Right pad (original): rows on P0,P1,P2,P7 / cols on P3,P4,P5,P6.
static const KeypadPins KP_PINS_RIGHT = {
  { 0xFE, 0xFD, 0xFB, 0x7F },   // drive P0,P1,P2,P7
  { 3, 4, 5, 6 },
  0x78
};
// Left pad (newer model): straight 4+4 split, P0-P3 / P4-P7.
static const KeypadPins KP_PINS_LEFT = {
  { 0xFE, 0xFD, 0xFB, 0xF7 },   // drive P0,P1,P2,P3
  { 4, 5, 6, 7 },
  0xF0
};

bool kpIsPressed(PCF8574 &pcf, const KeypadPins &p) {
  pcf.write8(p.readMask);  // all drive pins LOW, all read pins HIGH-Z
  return (pcf.read8() & p.readMask) != p.readMask;
}

uint8_t kpGetKey(PCF8574 &pcf, const KeypadPins &p) {
  for (uint8_t d = 0; d < 4; d++) {
    pcf.write8(p.driveWrite[d]);
    uint8_t val = pcf.read8();
    for (uint8_t r = 0; r < 4; r++) {
      if (!(val & (1 << p.readBit[r]))) {
        pcf.write8(0xFF);
        return d * 4 + r;
      }
    }
  }
  pcf.write8(0xFF);
  return 16;  // NoKey
}

// raw <= 100: noise floor; raw >= 65000: ADC overflow; 17000: pot physical max
int processPot(int raw, int out_max) {
  if (raw <= 100 || raw >= 65000) return 0;
  if (raw > 17000) raw = 17000;
  return map(raw, 0, 17000, 0, out_max);
}

// Centre-zero pot read: -half .. 0 .. +half instead of 0 .. 2*half, with the
// jitter suppression a stepper axis needs. A bare mapped ADC value wanders by
// a few counts even when nothing is touched, and every wander that survives
// the downstream threshold is a real motor step, so a "stationary" axis buzzes.
// Two filters, because they solve different halves of the problem:
//   DEADBAND    -- anything within this of centre reads exactly 0, so a control
//                  left at rest commands a hard stop rather than a small offset.
//   STICKY BAND -- elsewhere in travel, see stickyBand() below: holds still
//                  against noise but still tracks a slow move one count at a
//                  time. Entering the deadband always snaps to 0 so coming to
//                  rest never strands a small residual.
// `state` holds the last reported value and MUST be a distinct static/global
// per control -- sharing one would make the two pots fight over the filter.
//
// Unlike processPot() this does NOT treat a low reading as a noise floor.
// processPot()'s `raw <= 100 -> 0` guard is free on an unsigned scale, where 0
// is also the bottom of travel, but here 0 is the CENTRE: the same guard would
// make a pot turned fully to minimum snap to centre instead of -half, losing
// the bottom of its travel and jumping full-scale at the end stop. A missing
// module is already caught upstream by adsReady(), and a pivot pot that loses
// its supply leg reads as minimum and is caught by the axis MIN limit switch.
// Only the ADC's actual error signature (raw far above the ~17000 full-scale
// reading) still falls back to centre.
#define POT_DEADBAND    30   // +/- counts around centre that read as exactly 0
#define POT_STICKY_BAND  3   // see stickyBand(): noise immunity without quantising

// Sticky band -- kills at-rest jitter WITHOUT costing resolution while moving.
//
// The obvious filter ("only report a new value once it has moved N counts,
// then jump to it") does stop the jitter, but it also quantises real movement
// into N-sized steps: turn a pot slowly and the output hops N at a time
// instead of counting. That is unusable on an axis you want to inch.
//
// Instead the reported value TRAILS the live one by up to `band` and is
// dragged along rather than snapped:
//   at rest   -- noise inside +/-band never moves the report at all. Dead still.
//   moving    -- the report follows the input one count at a time, staying
//                `band` behind. Full 1-unit resolution is preserved.
// The only cost is `band` counts of lag, which on a 3200-count axis is well
// under a tenth of a percent of travel.
static inline int stickyBand(int v, int band, int &state) {
  if      (v - state >  band) state = v - band;
  else if (v - state < -band) state = v + band;
  return state;
}

// Spike rejection: median of the last three RAW samples.
//
// A dirty wiper momentarily reads somewhere else entirely, and one bad sample
// is enough to fling a servo or stepper across its travel. A median throws
// away ANY single-sample outlier completely, however far out it lands, while
// a genuine move still passes through -- unlike an averaging or slew-limiting
// filter, which would smear every fast move to soften the rare bad one.
//
// Cost is exactly one sample of lag (40ms at the 25Hz control rate), and only
// on direction changes; a spike has to repeat on two consecutive samples to
// get through. Filtered on the raw counts, upstream of everything else, so
// nothing downstream ever sees the spike.
struct Med3 { int s[3]; uint8_t n; };
int median3(Med3 &m, int v) {
  if (m.n < 3) { m.s[0] = m.s[1] = m.s[2] = v; m.n = 3; }   // first sight
  else         { m.s[0] = m.s[1]; m.s[1] = m.s[2]; m.s[2] = v; }
  int hi = max(m.s[0], max(m.s[1], m.s[2]));
  int lo = min(m.s[0], min(m.s[1], m.s[2]));
  return m.s[0] + m.s[1] + m.s[2] - hi - lo;   // the middle one
}

// One median3 state per control that drives motion (steppers and the eye
// servos). Not used on the face pots -- a stray frame on an eyebrow is
// cosmetic, where the same frame on the neck is the robot lurching.
Med3 med_neckx, med_necky, med_eyex, med_eyey, med_fl, med_fr;

// Map a raw ADC reading through a window defined by two MEASURED raw
// endpoints, clamping outside it. rawLo may be numerically greater than rawHi
// -- that is how an inverted (180-degree mounted) fader is expressed, so no
// separate invert flag is needed.
int mapWindow(int raw, int rawLo, int rawHi, int outLo, int outHi) {
  long v = (long)(raw - rawLo) * (outHi - outLo) / (rawHi - rawLo) + outLo;
  return constrain((int)v, min(outLo, outHi), max(outLo, outHi));
}

// Fader travel windows (raw ADS1115 counts).
//
// Each fader uses a window at the bottom of its mechanical travel: _RAW_BOTTOM
// reads the low end of the output range, _RAW_TOP the high end, the midpoint
// between them reads centre, and anything past _RAW_TOP stays pinned high.
//
// These are RAW endpoints rather than percentages of the ADC range on purpose.
// A fader's electrical span does not necessarily reach the ADC rails and does
// not necessarily cover its full mechanical travel, so the physical-to-raw
// relationship has to be measured, not assumed. Read them off the ADS_04
// screen (it shows raw counts): park the fader at each end of the window you
// want and note the count.
//
// fader_left is mounted inverted, hence BOTTOM > TOP.
//
// Measured 2026-08-30: fader_left reads 17560 at its physical bottom and
// fader_right reads 0 at its. 17560 counts is 3.29V, i.e. the faders do span
// the full 3.3V rail end to end.
//
// fader_left's window was then narrowed by hand to put centre on a chosen raw
// count: 17560 / 15000 / 12440 for -1600 / 0 / +1600. The two halves are the
// same width (2560 counts each), so the midpoint falls exactly on 15000. That
// window is 5120 counts, about 29% of the fader's travel rather than 40%, and
// the extra sensitivity is why fader_left needs its own sticky band below.
//
// fader_right still uses the bottom 40% of travel: 0 / 7024 for 0 / 255.
#define FADER_FULL_SCALE    17560   // raw at 100% of fader travel
#define FADER_L_RAW_BOTTOM  17560   // -> -1600  (bottom of travel)
#define FADER_L_RAW_CENTRE  15000   // ->     0  (midpoint, for reference)
#define FADER_L_RAW_TOP     12440   // -> +1600
#define FADER_L_OUT          1600
#define FADER_R_RAW_BOTTOM      0   // ->     0
#define FADER_R_RAW_TOP      7024   // ->   255
#define FADER_R_OUT           255

// fader_left's window is narrower than the neck axes' full-scale span, so the
// same raw noise turns into a bigger swing in output counts: about 1.6 raw
// counts per output count here against 5.3 on a neck axis. The ~8 raw counts
// of noise that show up as +/-1.5 counts on a neck axis are +/-5 counts here,
// so the shared POT_STICKY_BAND of 3 cannot hold this fader still and it needs
// its own, larger band.
//
// Simulated at 8 raw counts of noise across the window, the value stops moving
// at rest from a band of 5 upward; 7 leaves margin for noisier days and still
// costs only 0.22% of travel in lag, with a slow sweep resolving ~1150 distinct
// values. Raise it if the pivot buzzes at rest, lower it if the fader feels
// like it has slack near centre.
//
// The old, wider window was already marginal at the shared band of 3 (440
// value changes per 3000 reads at rest in the same simulation), so this is
// really a latent problem being fixed rather than one the narrowing created.
#define FADER_L_STICKY_BAND   7

// Eye joystick calibration, in processPot 0-255 units, measured on the bench
// 2026-08-30. The stick's electrical centre is not the midpoint of its travel
// and the two halves are not the same width, so each half is mapped
// separately -- a single straight map would put neutral off-zero and make one
// direction reach its endpoint before the other.
#define EYE_X_AT_MINUS  253   // left  extreme -> -128
#define EYE_X_AT_ZERO   135   // spring-centred
#define EYE_X_AT_PLUS    21   // right extreme -> +128
#define EYE_Y_AT_MINUS   13   // bottom extreme -> -128
#define EYE_Y_AT_ZERO   117   // spring-centred
#define EYE_Y_AT_PLUS   235   // top extreme -> +128
#define EYE_OUT         128   // full-scale magnitude either side of zero
#define EYE_DEADZONE      6   // output counts either side of 0 that read as 0
#define EYE_STICKY_BAND   1   // see stickyBand(); 1 count of a 256 span

// Map one joystick axis to -EYE_OUT..0..+EYE_OUT, each half on its own scale
// so `atZero` lands exactly on 0 and both extremes land exactly on full scale.
int mapAxisCentred(int v, int atMinus, int atZero, int atPlus, int out) {
  int r = (v == atZero) ? 0
        : (atPlus > atMinus)                        // which way the axis runs
            ? ((v > atZero) ? mapWindow(v, atZero, atPlus,  0,  out)
                            : mapWindow(v, atZero, atMinus, 0, -out))
            : ((v < atZero) ? mapWindow(v, atZero, atPlus,  0,  out)
                            : mapWindow(v, atZero, atMinus, 0, -out));
  if (abs(r) <= EYE_DEADZONE) r = 0;   // neutral must be dead silent
  return r;
}

// One eye axis, end to end. The deadzone SNAP is the important part: the
// sticky band trails its input by up to `band`, so on its own it will park
// one count off zero and sit there (that is what left the stick reading -1
// and +1 at rest). Inside the deadzone the state is forced to a hard 0
// instead of being dragged toward it; outside, the band does its usual job.
int eyeAxis(int raw, int atMinus, int atZero, int atPlus, int &state) {
  int v = mapAxisCentred(processPot(raw, 255), atMinus, atZero, atPlus, EYE_OUT);
  if (v == 0) state = 0;
  else        stickyBand(v, EYE_STICKY_BAND, state);
  return state;
}

int processPotCentred(int raw, int half, bool invert, int &state) {
  int v;
  if (raw >= 65000) {
    v = 0;                                  // ADC overflow / error -> centre
  } else {
    if (raw < 0)     raw = 0;
    if (raw > 17000) raw = 17000;
    v = map(raw, 0, 17000, -half, half);
    if (invert) v = -v;
    if (abs(v) < POT_DEADBAND) v = 0;
  }
  // In the deadband, snap to a hard 0. Elsewhere let the sticky band hold it
  // still at rest while still tracking a slow move count by count.
  if (v == 0) state = 0;
  else        stickyBand(v, POT_STICKY_BAND, state);
  return state;
}

// Dual-control arbitration: two pots drive one function (rotary + fader) and
// must never fight. Whichever control moved last (by more than the pair's
// claim threshold) becomes the active source and its value is used until the
// other one moves. A source that is not ok (module unplugged, display feed
// down) cannot claim or hold active status. First sighting of a source only
// baselines it -- it has to actually MOVE to claim, so power-up doesn't
// randomly hand control to a fader.
//
// The threshold is per-pair because the pairs are not all on the same scale:
// it has to be a similar FRACTION of travel on each, or the coarse-scale pair
// sits inside pot noise and flip-flops. ~1.5% of full travel on both.
#define DUAL_CLAIM_COUNTS   4   // 0-255 scale (face pots, iris)
// (a second, coarser threshold lived here for the neck-pivot pair; that pair
//  is gone now that the NECK PIV pot is disabled and fader_left is the only
//  source, so iris is the only pair left and only the 0-255 threshold is used)

// "Seen yet" is an explicit flag, not a -1 sentinel in lastA/lastB: the neck
// pivot pair is centre-zero now, so -1 is an ordinary value just left of
// centre. With the old sentinel any negative reading looked like a first
// sighting, and the pot could never claim back from the fader while it sat in
// the negative half of its travel.
struct DualPot {
  int  lastA, lastB;    // last reported value of each source
  bool seenA, seenB;    // false = not seen since it was last absent
  bool bActive;         // true = source B (the fader) is active
  int  claim;           // movement needed to take over, in this pair's units
};
DualPot dual_iris       = { 0, 0, false, false, false, DUAL_CLAIM_COUNTS };

int dualPick(DualPot &d, int a, bool aOk, int b, bool bOk) {
  // The ACTIVE source's baseline tracks its value; the INACTIVE source's
  // baseline stays frozen where it was released, so even a slow turn
  // accumulates enough travel to claim (per-cycle deltas never would).
  if (aOk) {
    if (!d.seenA || !d.bActive) { d.lastA = a; d.seenA = true; }  // first sight or active: track
    else if (abs(a - d.lastA) >= d.claim) { d.bActive = false; d.lastA = a; }
  } else {
    d.seenA = false;
  }
  if (bOk) {
    if (!d.seenB || d.bActive) { d.lastB = b; d.seenB = true; }   // first sight or active: track
    else if (abs(b - d.lastB) >= d.claim) { d.bActive = true; d.lastB = b; }
  } else {
    d.seenB = false;
  }
  if (d.bActive && !bOk) d.bActive = false;   // active source vanished
  if (!d.bActive && !aOk && bOk) d.bActive = true;
  return d.bActive ? b : a;
}

// I2C presence guards. Two traps when a device is missing:
//  1. The ADS1X15 library's readADC() has NO timeout: with no chip on the
//     bus, isBusy() never clears and readADC() spins forever, freezing
//     loop() (screen dead, button dead, no ESP-NOW).
//  2. With NO modules attached at all, the bus has no pull-ups (they live on
//     the breakout boards), so a transaction doesn't fast-NACK -- it eats the
//     driver timeout + bus recovery, hundreds of ms EACH. Probing every
//     absent device every cycle made loop() spend ~2s blocked per pass.
// So: transactions are capped short (Wire.setTimeOut in setup), a device
// must ACK before it is trusted, and an ABSENT device is only re-probed
// every I2C_REPROBE_MS. A present device ACKs in microseconds, so verifying
// it on every use costs nothing, and hot-plugged modules start working
// within a second.
#define I2C_REPROBE_MS 1000

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

struct AdsGuard {
  ADS1115 &ads;
  uint8_t addr;
  bool configured;
  unsigned long lastProbe;
  bool probedOnce;
  AdsGuard(ADS1115 &a, uint8_t adr)
    : ads(a), addr(adr), configured(false), lastProbe(0), probedOnce(false) {}
};

AdsGuard adsg_01(ADS_01, 0x48);
AdsGuard adsg_02(ADS_02, 0x49);
AdsGuard adsg_03(ADS_03, 0x4A);
AdsGuard adsg_04(ADS_04, 0x4B);

// Table driving the four ADS1115 screens (screen_mode 3-6), one row per
// module so adsPotsDisplay() doesn't need four near-duplicate functions.
//
// These show RAW ADS1115 counts, not the processed values -- raw is what you
// need to calibrate a fader window or a joystick centre, and this is the only
// place it is visible. Channels that are not read print "--".
// Each channel gets two lines: the raw count, and under it what that channel
// drives plus the processed value that actually leaves this board. A null
// value pointer (a disabled or faulty channel) prints "--".
//
// The two neck channels are the one place the pairing is not one-to-one: X
// and Y are mixed into neck-L and neck-R together, so rather than invent a
// per-axis destination the screen shows one mixed output under each.
// guard is null for the module that is not on this board's I2C bus: the fifth
// ADS1115 lives on j4_display_right, so its "present" test is the freshness of
// that board's serial feed rather than an I2C ACK.
struct AdsPotScreen {
  AdsGuard   *guard;
  const char *title;
  uint8_t     module;        // index into ads_raw[][]
  const char *labels[4];     // channel name, beside the raw count
  const char *dests[4];      // what it drives, beside the processed value
  int        *values[4];     // processed value (nullptr = channel not in use)
};

AdsPotScreen adsPotScreens[5] = {
  { &adsg_01, "ADS_01  0x48", 0,
    { "NECK X", "NECK Y", "EYES X", "EYES Y" },
    { "NECK-L", "NECK-R", "EYE-X",  "EYE-Y"  },
    { &neck_left_value, &neck_right_value, &eyes_x_value, &eyes_y_value } },
  { &adsg_02, "ADS_02  0x49", 1,
    { "BROW L", "BROW R", "BBRW L", "BBRW R" },
    { "PCA 6",  "PCA 7",  "PCA 8",  "PCA 9"  },
    { &eyebrow_l_value, &eyebrow_r_value, &basket_brow_l_value, &basket_brow_r_value } },
  { &adsg_03, "ADS_03  0x4A", 2,
    { "NOSE",   "A1 FAULT", "EYELID L", "EYELID R" },
    { "PCA 10", "OFF",      "PCA 12",   "PCA 13"   },
    { &nose_value, nullptr, &eyelid_l_value, &eyelid_r_value } },
  { &adsg_04, "ADS_04  0x4B", 3,
    { "PIV OFF", "FADER L",  "FADER R", "NOSE BK" },
    { "OFF",     "NECK PIV", "IRIS",    "PCA 11"  },
    { nullptr, &neck_pivot_value, &iris_value, &nose_basket_value } },
  // The fifth ADS1115: on j4_display_right, not on this board's bus. Raw
  // counts arrive over Serial2 as "P:" lines, so CONNECTED here means that
  // feed is fresh. IRIS is the arbitrated result and can disagree with this
  // pot's own raw when fader_right is the active source -- that disagreement
  // is the quickest way to see which one currently owns the channel.
  { nullptr, "DISP_R  0x48", 4,
    { "IRIS",  "COLOR",  "BRIGHT", "VOLUME" },
    { "PCA 5", "WS2812", "WS2812", "TALK"   },
    { &iris_value, &color_value, &brightness_value, &volume_value } },
};

bool adsReady(AdsGuard &g) {
  if (!g.configured) {   // absent (or never seen): probe only once a second
    unsigned long now = millis();
    if (g.probedOnce && (now - g.lastProbe < I2C_REPROBE_MS)) return false;
    g.probedOnce = true;
    g.lastProbe  = now;
  }
  if (!i2cPresent(g.addr)) {
    g.configured = false;
    return false;
  }
  if (!g.configured) {
    g.ads.begin();
    g.ads.setGain(0);      //  0 is ±6.144V    1 is ±4.096V    2 is ±2.048V
    g.ads.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
    g.ads.setMode(1);      //  0 = continuous mode   1 = single mode
    g.ads.requestADC(0);   //  first read to trigger
    g.configured = true;
  }
  return true;
}

// Same guard for the PCF8574 keypad expanders: absent, their floating-bus
// reads look like a key held down forever, so no scan unless the chip ACKs.
struct KeypadGuard {
  PCF8574          &pcf;
  uint8_t           addr;
  const char       *keymap;
  const KeypadPins &pins;   // this pad's drive/read split (models differ)
  bool              present;
  unsigned long     lastProbe;
  bool              probedOnce;
  KeypadGuard(PCF8574 &p, uint8_t a, const char *km, const KeypadPins &kp)
    : pcf(p), addr(a), keymap(km), pins(kp),
      present(false), lastProbe(0), probedOnce(false) {}
};

KeypadGuard kpg_left (pcf_left,  0x21, keymap_left,  KP_PINS_LEFT);
KeypadGuard kpg_right(pcf_right, 0x20, keymap_right, KP_PINS_RIGHT);

bool keypadReady(KeypadGuard &g) {
  if (!g.present) {   // absent (or never seen): probe only once a second
    unsigned long now = millis();
    if (g.probedOnce && (now - g.lastProbe < I2C_REPROBE_MS)) return false;
    g.probedOnce = true;
    g.lastProbe  = now;
  }
  g.present = i2cPresent(g.addr);
  return g.present;
}


// --- FACE PRESET HELPERS ---

// The talk chain is usable when the receiver's status packets are fresh AND
// the receiver reports the Teensy alive. Never a blocking check.
bool talkLinkUp() {
  return (millis() - lastStatusRecvMs < STATUS_LINK_TIMEOUT_MS)
      && rcvData.talk_ok_rcv;
}

FaceSlot *faceFind(char kc) {
  for (uint8_t i = 0; i < FACE_SLOTS; i++)
    if (faces[i].key == kc) return &faces[i];
  return NULL;
}

FaceSlot *faceAlloc(char kc) {
  FaceSlot *f = faceFind(kc);
  if (f) return f;
  for (uint8_t i = 0; i < FACE_SLOTS; i++)
    if (faces[i].key == 0) return &faces[i];
  return NULL;   // cannot happen: 15 possible keys, 16 slots
}

// Message on j4_display_right (Serial2 TX). showMs = 0 leaves it up until
// replaced or cleared; otherwise loop() sends "X:" after showMs. Fire and
// forget: if the display is unplugged the bytes just fall on the floor, and
// the display self-clears a stale message after 15s anyway.
void faceMsg(const char *l1, const char *l2, unsigned long showMs) {
  Serial2.printf("M:%s|%s\n", l1, l2);
  faceMsg_clearAt = showMs ? millis() + showMs : 0;
}

void facePromptOpen(char kc) {
  face_prompt_key     = kc;
  face_prompt_started = millis();
  char l1[24];
  snprintf(l1, sizeof(l1), "SAVE FACE ON %c?", kc);
  faceMsg(l1, "PRESS * TO CONFIRM", 0);
}

void facePromptCancel(const char *why) {
  face_prompt_key = 0;
  faceMsg("SAVE CANCELLED", why, 2500);
}

// Snapshot the CURRENT face -- the effective values (post arbitration and
// preset overlay), i.e. exactly what the robot's face looks like right now.
void faceSaveConfirmed() {
  char kc = face_prompt_key;
  face_prompt_key = 0;
  FaceSlot *f = faceAlloc(kc);
  if (!f) { faceMsg("SAVE FAILED", "NO FREE SLOTS", 2500); return; }
  f->key       = kc;
  f->valid     = true;
  f->dirty     = true;
  f->sd_failed = false;
  f->v[FI_IRIS]        = iris_value;
  f->v[FI_COLOR]       = color_value;
  f->v[FI_BRIGHT]      = brightness_value;
  f->v[FI_BROW_L]      = eyebrow_l_value;
  f->v[FI_BROW_R]      = eyebrow_r_value;
  f->v[FI_BBROW_L]     = basket_brow_l_value;
  f->v[FI_BBROW_R]     = basket_brow_r_value;
  f->v[FI_NOSE]        = nose_value;
  f->v[FI_NOSE_BASKET] = nose_basket_value;
  f->v[FI_LID_L]       = eyelid_l_value;
  f->v[FI_LID_R]       = eyelid_r_value;
  f->toggles = (laser_toggle   << TOGGLE_BIT_LASER)
             | (vent_toggle    << TOGGLE_BIT_VENT)
             | (eye_pop_toggle << TOGGLE_BIT_EYE_POP);
  char l1[24];
  snprintf(l1, sizeof(l1), "FACE SAVED ON %c", kc);
  faceMsg(l1, talkLinkUp() ? "WRITING TO SD..." : "PENDING SD: TALK OFFLINE", 2500);
}

void faceRecall(char kc) {
  char l1[24];
  FaceSlot *f = faceFind(kc);
  if (!f || !f->valid) {
    snprintf(l1, sizeof(l1), "NO FACE ON %c", kc);
    faceMsg(l1, "HOLD 3s TO SAVE ONE", 2500);
    return;
  }
  for (uint8_t i = 0; i < FACE_VALUES; i++) {
    preset_v[i]  = f->v[i];
    preset_on[i] = true;
  }
  preset_toggles     = f->toggles;
  preset_toggle_mask = (1 << TOGGLE_BIT_LASER) | (1 << TOGGLE_BIT_VENT)
                     | (1 << TOGGLE_BIT_EYE_POP);
  // phys_baseline stops updating while preset_on, so takeover measures total
  // travel since this exact moment.
  snprintf(l1, sizeof(l1), "FACE %c RECALLED", kc);
  faceMsg(l1, "MOVE A POT TO RETAKE IT", 2000);
}

void faceSendPkt(uint8_t op, const FaceSlot *f) {
  espnow_face_pkt_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.pkt_type = ESPNOW_PKT_FACE;
  pkt.op       = op;
  if (f) {
    pkt.key     = (uint8_t)f->key;
    pkt.toggles = f->toggles;
    memcpy(pkt.v, f->v, sizeof(pkt.v));
  }
  esp_now_send(broadcastAddress, (uint8_t *)&pkt, sizeof(pkt));
}

// Drain face packets staged by OnDataRecv. Runs in loop context only.
void processFacePackets() {
  char l1[24];
  while (faceRxTail != faceRxHead) {
    espnow_face_pkt_t pkt;
    memcpy(&pkt, (const void *)&faceRxQ[faceRxTail], sizeof(pkt));
    faceRxTail = (uint8_t)((faceRxTail + 1) % FACE_RXQ);

    if (pkt.op == FACE_OP_DATA) {
      faceDumpCount++;
      FaceSlot *f = faceAlloc((char)pkt.key);
      // A locally-dirty slot is newer than the SD copy -- keep ours, it will
      // be pushed by the dirty timer and come back clean.
      if (f && !f->dirty) {
        f->key     = (char)pkt.key;
        f->valid   = true;
        f->dirty   = false;
        f->sd_failed = false;
        f->toggles = pkt.toggles;
        memcpy(f->v, pkt.v, sizeof(f->v));
      }

    } else if (pkt.op == FACE_OP_END) {
      // Complete only if every face in the dump actually landed; otherwise
      // stay unsynced and the REQ timer asks again (idempotent).
      faces_synced = (faceDumpCount == (uint8_t)pkt.v[0]);

    } else if (pkt.op == FACE_OP_ACK) {
      FaceSlot *f = faceFind((char)pkt.key);
      if (f) f->dirty = false;
      snprintf(l1, sizeof(l1), "FACE %c ON SD", (char)pkt.key);
      faceMsg(l1, "", 1500);

    } else if (pkt.op == FACE_OP_ERR) {
      FaceSlot *f = faceFind((char)pkt.key);
      if (f) f->sd_failed = true;   // keep valid + dirty, stop auto-retrying
      faceMsg("SD WRITE FAILED", "FACE KEPT IN CONTROLLER", 3000);
    }
  }
}
// --- END FACE PRESET HELPERS ---


void setup() {
  Wire.begin(SDA, SCL);
  // Cap each I2C transaction. With no modules attached the bus has no
  // pull-ups and a transaction eats driver timeout + recovery instead of a
  // fast NACK; uncapped that is hundreds of ms per attempt. A healthy
  // transaction completes in well under 1ms, so 10ms is generous.
  Wire.setTimeOut(10);

  pcf_left.begin();    // both keypad expanders; fine if either is absent
  pcf_right.begin();
  Serial.begin(115200);

  // Probe + configure whichever ADS1115 modules are actually on the bus.
  // Missing ones are fine: their channels read 0 and they are re-probed
  // every I2C_REPROBE_MS, so plugging one in just starts working.
  adsReady(adsg_01);
  adsReady(adsg_02);
  adsReady(adsg_03);
  adsReady(adsg_04);  // A0 neck-pivot, A1 fader_left, A2 fader_right, A3 spare

  // Panel toggles: switch closes to GND, so ON reads LOW
  pinMode(LASER_TOGGLE_PIN,   INPUT_PULLUP);
  pinMode(VENT_TOGGLE_PIN,    INPUT_PULLUP);
  pinMode(EYE_POP_TOGGLE_PIN, INPUT_PULLUP);
  pinMode(AUX_TOGGLE_PIN,     INPUT_PULLUP);

  // XIAO display links - init after ADS1115 to avoid I2C/UART peripheral conflict
  Serial1.begin(XIAO_BAUD, SERIAL_8N1, XIAO_RX_PIN,  XIAO_TX_PIN);    // j4_display_left
  Serial2.begin(XIAO_BAUD, SERIAL_8N1, XIAOR_RX_PIN, XIAOR_TX_PIN);   // j4_display_right

  WiFi.mode(WIFI_STA);
  Serial.println("MAC Address: ");
  Serial.println(WiFi.macAddress());
  WiFi.setSleep(false);

  // RF hardening for crowded 2.4GHz venues (must match j4_receiver):
  // LR = Espressif's proprietary long-range PHY. Both ends are ESP32s, so the
  // link gains ~4dB sensitivity and ordinary WiFi gear cannot even decode it.
  // Max TX power = 19.5dBm (units of 0.25dBm).
  // Channel 6 (not the power-on default of 1) dodges every other ESP-NOW
  // project left on its default channel. Both ends must match; if the venue
  // is ugly on 6, change ESPNOW_CHANNEL on BOTH boards (1/6/11 only) + reflash.
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  esp_wifi_set_max_tx_power(78);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    connectStatus = "init error";
    connectError = HIGH;
    return;
  }
  connectStatus = "init OK";

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    connectStatus = "no peer added";
    connectError = HIGH;
    return;
  }
  connectStatus = "peer added";

  xmitData.pkt_type = ESPNOW_PKT_CONTROL;
  xmitData.phrase_select_xmit[0] = '\0';

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);

  screen_bottom_sprite_203.createSprite(135, 203);
  tft.pushImage(0, 0, 135, 37, kevco_labs_logo_02);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN, TFT_BLACK);
  labelsDisplaySprite();
  screen_bottom_sprite_203.drawString("Ready...", 0, 0, 1);

  pinMode(SCREEN_BUTTON, INPUT_PULLUP);   // TTGO built-in button (GPIO 35)
}


void loop() {
  unsigned long currentMillis = millis();

  controllerScreenModeDetect();

  if (currentMillis - tft_update_previousMillis >= tft_update_interval) {
    tft_update_previousMillis = currentMillis;
    tftDisplayUpdate();
    sendToXIAO();
  }

  // XIAO display: file-list requests + heartbeat ("PING") for connection status
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      xiaoSerialBuf.trim();
      // Only lines j4_display_left actually sends count as its heartbeat --
      // with the display unplugged the floating RX pin generates garbage
      // lines, which used to fake a CONNECTED status.
      if (xiaoSerialBuf == "PING" || xiaoSerialBuf == "LIST?") lastDisplayMs = millis();
      if (xiaoSerialBuf == "LIST?" && jukeboxReady) sendFileListToXIAO();
      xiaoSerialBuf = "";
    } else if (c != '\r') {
      if (xiaoSerialBuf.length() > 16) xiaoSerialBuf = "";  // line noise guard
      xiaoSerialBuf += c;
    }
  }

  // j4_display_right: "P:<iris>,<color>,<brightness>,<volume>" pot feed (raw
  // ADS1115 counts from its dedicated ADS1115). Any valid line = board alive.
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      int i, col, b, v;
      if (sscanf(xiaoRSerialBuf.c_str(), "P:%d,%d,%d,%d", &i, &col, &b, &v) == 4) {
        dispR_iris_raw       = i;
        dispR_color_raw      = col;
        dispR_brightness_raw = b;
        dispR_volume_raw     = v;
        lastDisplayRMs = millis();
      }
      xiaoRSerialBuf = "";
    } else if (c != '\r') {
      if (xiaoRSerialBuf.length() > 40) xiaoRSerialBuf = "";  // line noise guard
      xiaoRSerialBuf += c;
    }
  }

  // --- CONTROL TX (25 Hz): read inputs, fill the packet, send ---
  // This whole block MUST stay rate-limited. When it ran every loop pass the
  // pass rate was only held down by the blocking ADC reads; with no ADCs
  // attached the loop spun at multi-kHz, esp_now_send() fired thousands of
  // times a second, and the OnDataSent callback storm corrupted the heap
  // (lockup ~2s after boot). 25 Hz matches the receiver's 20ms packet gate.
  if (currentMillis - control_tx_previousMillis >= control_tx_interval) {
  control_tx_previousMillis = currentMillis;

  // --- ADC READS ---
  // Each module is probed before its channels are read: readADC() on an
  // absent chip never returns (see adsReady()). ADS_04 (0x4B) is spare.
  if (adsReady(adsg_01)) {
    // Every channel goes through median3() on the raw counts first, so a
    // single dirty-wiper spike never reaches a servo or stepper.
    ads_raw[0][0] = median3(med_neckx, ADS_01.readADC(0));   // neck joystick X
    ads_raw[0][1] = median3(med_necky, ADS_01.readADC(1));   // neck joystick Y
    ads_raw[0][2] = median3(med_eyex,  ADS_01.readADC(2));   // eyes joystick X
    ads_raw[0][3] = median3(med_eyey,  ADS_01.readADC(3));   // eyes joystick Y
    // The two neck axes are sticky-filtered at the source, before the mixer,
    // so neck-L and neck-R both come out steady. Filtering the mixer outputs
    // instead would leave each axis free to jitter into the other.
    neck_value   = stickyBand(processPot(ads_raw[0][0], 3200), POT_STICKY_BAND, neck_sticky);
    jaw_value    = stickyBand(processPot(ads_raw[0][1], 3200), POT_STICKY_BAND, jaw_sticky);
    // Eyes are centre-zero (-128..0..128) with a hard zero at neutral.
    eyes_x_value = eyeAxis(ads_raw[0][2], EYE_X_AT_MINUS, EYE_X_AT_ZERO, EYE_X_AT_PLUS, eyex_sticky);
    eyes_y_value = eyeAxis(ads_raw[0][3], EYE_Y_AT_MINUS, EYE_Y_AT_ZERO, EYE_Y_AT_PLUS, eyey_sticky);
  } else {
    eyes_x_value = eyes_y_value = 0;      // centre-zero: 0 = eyes centred
    eyex_sticky  = eyey_sticky  = 0;
    neck_value   = jaw_value    = 1600;   // joystick centre, not hard-over
    neck_sticky  = jaw_sticky   = 1600;   // reset the filters with them
    ads_raw[0][0] = ads_raw[0][1] = ads_raw[0][2] = ads_raw[0][3] = -1;
  }
  if (adsReady(adsg_02)) {
    ads_raw[1][0] = ADS_02.readADC(0);
    ads_raw[1][1] = ADS_02.readADC(1);
    ads_raw[1][2] = ADS_02.readADC(2);
    ads_raw[1][3] = ADS_02.readADC(3);
    eyebrow_l_value     = processPot(ads_raw[1][0], 255);  // Eyebrow L
    eyebrow_r_value     = processPot(ads_raw[1][1], 255);  // Eyebrow R
    basket_brow_l_value = processPot(ads_raw[1][2], 255);  // Basket Eyebrow L
    basket_brow_r_value = processPot(ads_raw[1][3], 255);  // Basket Eyebrow R
  } else {
    eyebrow_l_value = eyebrow_r_value = basket_brow_l_value = basket_brow_r_value = 0;
    ads_raw[1][0] = ads_raw[1][1] = ads_raw[1][2] = ads_raw[1][3] = -1;
  }
  if (adsReady(adsg_03)) {
    ads_raw[2][0] = ADS_03.readADC(0);
    ads_raw[2][1] = -1;   // A1 is faulty on this module and is left unread
    ads_raw[2][2] = ADS_03.readADC(2);
    ads_raw[2][3] = ADS_03.readADC(3);
    nose_value          = processPot(ads_raw[2][0], 255);  // Nose (up/down)
    eyelid_l_value      = processPot(ads_raw[2][2], 255);  // Bottom Eyelid L
    eyelid_r_value      = processPot(ads_raw[2][3], 255);  // Bottom Eyelid R
  } else {
    nose_value = eyelid_l_value = eyelid_r_value = 0;
    ads_raw[2][0] = ads_raw[2][1] = ads_raw[2][2] = ads_raw[2][3] = -1;
  }
  bool ads4_ok = adsReady(adsg_04);
  if (ads4_ok) {
    // A0 is the old NECK PIV pot: DISABLED as a control, but still read so the
    // ADS_04 screen can show it while it waits to be repurposed. The neck
    // pivot is driven by fader_left alone now.
    ads_raw[3][0] = ADS_04.readADC(0);
    ads_raw[3][1] = median3(med_fl, ADS_04.readADC(1));
    ads_raw[3][2] = median3(med_fr, ADS_04.readADC(2));
    ads_raw[3][3] = ADS_04.readADC(3);
    // Both faders use only the bottom 40% of their travel, mapped through
    // measured raw endpoints. fader_left is centre-zero and mounted inverted
    // (its BOTTOM endpoint is the numerically higher raw count).
    fader_left_value  = stickyBand(mapWindow(ads_raw[3][1],
                                             FADER_L_RAW_BOTTOM, FADER_L_RAW_TOP,
                                             -FADER_L_OUT, FADER_L_OUT),
                                   FADER_L_STICKY_BAND, fl_sticky);
    fader_right_value = mapWindow(ads_raw[3][2],
                                  FADER_R_RAW_BOTTOM, FADER_R_RAW_TOP,
                                  0, FADER_R_OUT);        // fader_right -> iris
    nose_basket_value = processPot(ads_raw[3][3], 255);   // Nose Basket
  } else {
    fader_left_value  = 0;   // centre-zero scale: 0 = stop
    fl_sticky         = 0;   // reset the filter with it
    fader_right_value = nose_basket_value = 0;
    ads_raw[3][0] = ads_raw[3][1] = ads_raw[3][2] = ads_raw[3][3] = -1;
  }
  // Neck pivot now has exactly one source: fader_left. The NECK PIV pot is
  // disabled, so there is no pair left to arbitrate.
  neck_pivot_value = fader_left_value;
  // Remote pots from j4_display_right (same raw scale -> same processPot)
  bool dispR_ok    = (millis() - lastDisplayRMs < DISPLAY_TIMEOUT_MS);
  // Mirror those raw counts into the screen table as module 4. Stale feed
  // shows "--" rather than the last values frozen on screen looking live.
  if (dispR_ok) {
    ads_raw[4][0] = dispR_iris_raw;
    ads_raw[4][1] = dispR_color_raw;
    ads_raw[4][2] = dispR_brightness_raw;
    ads_raw[4][3] = dispR_volume_raw;
  } else {
    ads_raw[4][0] = ads_raw[4][1] = ads_raw[4][2] = ads_raw[4][3] = -1;
  }
  iris_value       = processPot(dispR_iris_raw, 255);
  color_value      = processPot(dispR_color_raw, 255);
  brightness_value = processPot(dispR_brightness_raw, 255);
  volume_value     = processPot(dispR_volume_raw, 100);
  // Dual-control arbitration: iris = IRIS pot vs fader_right. Last mover
  // wins; see dualPick(). The neck pivot no longer has a pair -- the NECK PIV
  // pot is disabled and fader_left is its only source -- and Nose Basket is
  // single-source too, so iris is the only pair left.
  iris_value        = dualPick(dual_iris, iris_value, dispR_ok,
                               fader_right_value, ads4_ok);
  // --- END ADC READS ---

  // Panel toggles: INPUT_PULLUP, switch closes to GND, so ON = LOW
  laser_toggle   = (digitalRead(LASER_TOGGLE_PIN)   == LOW);
  vent_toggle    = (digitalRead(VENT_TOGGLE_PIN)    == LOW);
  eye_pop_toggle = (digitalRead(EYE_POP_TOGGLE_PIN) == LOW);
  aux_toggle     = (digitalRead(AUX_TOGGLE_PIN)     == LOW);

  // --- FACE PRESET OVERLAY ---
  // A recalled face holds each channel until that channel's own physical
  // control moves, and each toggle until its switch is flipped. Baselines
  // only track while no preset holds the channel, so takeover measures
  // total travel since the recall (a slow turn still gets there).
  {
    int16_t phys[FACE_VALUES] = {
      (int16_t)iris_value, (int16_t)color_value, (int16_t)brightness_value,
      (int16_t)eyebrow_l_value, (int16_t)eyebrow_r_value,
      (int16_t)basket_brow_l_value, (int16_t)basket_brow_r_value,
      (int16_t)nose_value, (int16_t)nose_basket_value,
      (int16_t)eyelid_l_value, (int16_t)eyelid_r_value
    };
    uint8_t tphys = (laser_toggle   << TOGGLE_BIT_LASER)
                  | (vent_toggle    << TOGGLE_BIT_VENT)
                  | (eye_pop_toggle << TOGGLE_BIT_EYE_POP);
    if (!phys_baseline_init) {
      memcpy(phys_baseline, phys, sizeof(phys_baseline));
      toggles_phys_last  = tphys;
      phys_baseline_init = true;
    }
    for (uint8_t i = 0; i < FACE_VALUES; i++) {
      if (preset_on[i] && abs(phys[i] - phys_baseline[i]) >= DUAL_CLAIM_COUNTS)
        preset_on[i] = false;                     // pot moved: it takes over
      if (!preset_on[i]) phys_baseline[i] = phys[i];
    }
    preset_toggle_mask &= ~(tphys ^ toggles_phys_last);   // flipped = reclaimed
    toggles_phys_last = tphys;

    if (preset_on[FI_IRIS])        iris_value          = preset_v[FI_IRIS];
    if (preset_on[FI_COLOR])       color_value         = preset_v[FI_COLOR];
    if (preset_on[FI_BRIGHT])      brightness_value    = preset_v[FI_BRIGHT];
    if (preset_on[FI_BROW_L])      eyebrow_l_value     = preset_v[FI_BROW_L];
    if (preset_on[FI_BROW_R])      eyebrow_r_value     = preset_v[FI_BROW_R];
    if (preset_on[FI_BBROW_L])     basket_brow_l_value = preset_v[FI_BBROW_L];
    if (preset_on[FI_BBROW_R])     basket_brow_r_value = preset_v[FI_BBROW_R];
    if (preset_on[FI_NOSE])        nose_value          = preset_v[FI_NOSE];
    if (preset_on[FI_NOSE_BASKET]) nose_basket_value   = preset_v[FI_NOSE_BASKET];
    if (preset_on[FI_LID_L])       eyelid_l_value      = preset_v[FI_LID_L];
    if (preset_on[FI_LID_R])       eyelid_r_value      = preset_v[FI_LID_R];
    if (preset_toggle_mask & (1 << TOGGLE_BIT_LASER))
      laser_toggle   = (preset_toggles >> TOGGLE_BIT_LASER)   & 1;
    if (preset_toggle_mask & (1 << TOGGLE_BIT_VENT))
      vent_toggle    = (preset_toggles >> TOGGLE_BIT_VENT)    & 1;
    if (preset_toggle_mask & (1 << TOGGLE_BIT_EYE_POP))
      eye_pop_toggle = (preset_toggles >> TOGGLE_BIT_EYE_POP) & 1;
  }
  // --- END FACE PRESET OVERLAY ---

  eye_pop_value  = eye_pop_toggle ? 3200 : 0;  // popped / normal

  // Dead zone: snap joystick axes to center if within threshold
  if (abs(neck_value - 1600) <= JOYSTICK_DEAD_ZONE) neck_value = 1600;
  if (abs(jaw_value  - 1600) <= JOYSTICK_DEAD_ZONE) jaw_value  = 1600;

  // Neck mixer: Y sets base height, X steers left/right differentially.
  // Both outputs are centre-zero (-1600..0..1600), so both joystick axes are
  // offset to centre-zero before mixing rather than after; mixing in the
  // 0-3200 domain and subtracting afterwards would clamp at the wrong ends.
  neck_left_value  = constrain((jaw_value - 1600) + (neck_value - 1600), -1600, 1600);
  neck_right_value = constrain((jaw_value - 1600) - (neck_value - 1600), -1600, 1600);

  xmitData.volume_xmit      = volume_value;
  xmitData.iris_xmit        = iris_value;
  xmitData.color_xmit       = color_value;
  xmitData.brightness_xmit  = brightness_value;
  // Eyes are centre-zero (-128..128) inside this board; the wire stays the
  // 0-255 the receiver maps straight onto the pan/tilt servo pulse range.
  // Neutral now lands on 128, i.e. true servo centre (it used to sit at the
  // stick's electrical centre of 135, slightly off).
  xmitData.eyes_x_xmit      = constrain(eyes_x_value + 128, 0, 255);
  xmitData.eyes_y_xmit      = constrain(eyes_y_value + 128, 0, 255);
  xmitData.eye_pop_xmit     = eye_pop_value;
  // neck-L/R are centre-zero here, but the wire stays the 0-3200 absolute
  // that j4_stepper_neck turns into an absolute position (nL/nR halved, homed
  // off the MIN limit switches). Same reasoning as nP below: signed on the
  // wire would command negative positions and drive into the limit switches.
  xmitData.neck_left_xmit   = constrain(neck_left_value  + 1600, 0, 3200);
  xmitData.neck_right_xmit  = constrain(neck_right_value + 1600, 0, 3200);
  // neck_pivot is centre-zero (-2048..2048, straight off fader_left) inside
  // this board, but the wire format stays the absolute 0-3200 the receiver
  // forwards and j4_stepper_neck turns into an absolute position (nP/2, homed
  // off the MIN limit switch). Sending the signed value raw would command
  // negative positions and drive the pivot into its MIN limit, so rescale and
  // re-centre here rather than changing the packet contract on three boards.
  xmitData.neck_pivot_xmit  = constrain(map(neck_pivot_value, -FADER_L_OUT, FADER_L_OUT,
                                            0, 3200), 0, 3200);
  xmitData.eyebrow_l_xmit     = eyebrow_l_value;
  xmitData.eyebrow_r_xmit     = eyebrow_r_value;
  xmitData.basket_brow_l_xmit = basket_brow_l_value;
  xmitData.basket_brow_r_xmit = basket_brow_r_value;
  xmitData.nose_xmit          = nose_value;
  xmitData.nose_basket_xmit   = nose_basket_value;
  xmitData.eyelid_l_xmit      = eyelid_l_value;
  xmitData.eyelid_r_xmit      = eyelid_r_value;
  xmitData.toggles_xmit       = (laser_toggle   << TOGGLE_BIT_LASER)
                              | (vent_toggle    << TOGGLE_BIT_VENT)
                              | (eye_pop_toggle << TOGGLE_BIT_EYE_POP)
                              | (aux_toggle     << TOGGLE_BIT_AUX);
  xmitData.need_filelist_xmit = jukeboxReady ? 0 : 1;
  xmitData.display_l_ok_xmit  = (millis() - lastDisplayMs  < DISPLAY_TIMEOUT_MS) ? 1 : 0;
  xmitData.display_r_ok_xmit  = (millis() - lastDisplayRMs < DISPLAY_TIMEOUT_MS) ? 1 : 0;

  esp_now_send(broadcastAddress, (uint8_t *)&xmitData, sizeof(xmitData));

  // Render the last send result (flag set by OnDataSent in the WiFi task)
  // into the display String from loop context only -- String ops inside the
  // callback raced loop()'s heap use and corrupted it.
  connectStatus = connectError ? "xmit failed" : "xmit success";
  }
  // --- END CONTROL TX ---

  // File list completed by OnDataRecv (WiFi task): forward it to the display
  // from here so Serial1 is only ever written from loop context.
  if (filelistForwardPending) {
    filelistForwardPending = false;
    sendFileListToXIAO();
  }

  // --- BATTERY RELATED ---
  if (currentMillis - battery_01_previousMillis >= battery_01_interval) {
    battery_01_previousMillis = currentMillis;

    float product = 0.0018276 * analogRead(34);  // IO34 is battery voltage
    bat1_mv = (uint16_t)(product * 1000.0f);

    char str[10];
    dtostrf(product, 4, 2, str);

    String voltage_battery_01 = String(str) + "V";
    String voltage_battery_02 = "6.00V";   // PLACEHOLDER
    String voltage_battery_03 = "12.00V";  // PLACEHOLDER

    screen_bottom_sprite_203.setTextColor(TFT_BLACK);
    screen_bottom_sprite_203.fillRect( 0, 185, 40, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_01,  2, 185, 2);
    screen_bottom_sprite_203.fillRect(43, 185, 40, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_02, 45, 185, 2);
    screen_bottom_sprite_203.fillRect(86, 185, 49, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_03, 88, 185, 2);
  }
  // --- END BATTERY RELATED ---

  if (currentMillis - keypad_previousMillis >= keypad_interval) {
    keypad_previousMillis = currentMillis;

    // LEFT keypad -- phrase select (jukebox). The right keypad became the
    // face keypad in v0_6_23 and is scanned separately below.
    KeypadGuard &pad = kpg_left;
    if (keypadReady(pad) && kpIsPressed(pad.pcf, pad.pins)) {
      uint8_t rawKey = kpGetKey(pad.pcf, pad.pins);

      if (rawKey <= 15) {  // 16 = NoKey - only promote to global on valid read
      key = (int)rawKey;

      if (ready_message) {
        screen_bottom_sprite_203.fillRect(0, 0, 135, 20, TFT_BLACK);
        ready_message = false;
      } else {
        screen_bottom_sprite_203.fillRect(70, 0, 65, 20, TFT_BLACK);
      }

      if (key != old_key) {
        char kc = pad.keymap[key];
        last_key_char = kc;

        if (kc == '#') {  // reset same-key lock and clear buffer
          old_key = -1;
          phrase_select_buffer = "";
          screen_bottom_sprite_203.fillRect(70,  0, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);

        } else if (kc == '*') {  // stop playback
          old_key = -1;
          phrase_select_buffer = "STOP";
          strncpy(xmitData.phrase_select_xmit, phrase_select_buffer.c_str(), sizeof(xmitData.phrase_select_xmit) - 1);
          xmitData.phrase_select_xmit[sizeof(xmitData.phrase_select_xmit) - 1] = '\0';
          screen_bottom_sprite_203.setTextColor(TFT_GREEN);
          screen_bottom_sprite_203.fillRect(70,  0, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.drawString("*",                  70,  0, 2);
          screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.drawString(phrase_select_buffer, 70, 20, 2);
          phrase_select_buffer = "";

        } else {
          old_key = key;

          if (kc >= 'A' && kc <= 'D') {  // Letter prefix - wait for digit
            phrase_select_buffer = String(kc);
            screen_bottom_sprite_203.setTextColor(TFT_GREEN);
            screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
            screen_bottom_sprite_203.drawString(String(kc) + "_", 70, 20, 2);
          } else {
            if (phrase_select_buffer.length() == 0) phrase_select_buffer = "0";
            phrase_select_buffer += String(kc);

            screen_bottom_sprite_203.setTextColor(TFT_GREEN);
            screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
            screen_bottom_sprite_203.drawString(phrase_select_buffer + ".wav", 70, 20, 2);

            strncpy(xmitData.phrase_select_xmit, phrase_select_buffer.c_str(), sizeof(xmitData.phrase_select_xmit) - 1);
            xmitData.phrase_select_xmit[sizeof(xmitData.phrase_select_xmit) - 1] = '\0';
            phrase_select_buffer = "";
          }
        }
      }
      }  // end key <= 15 guard
    }

    // RIGHT keypad -- FACE PRESETS. Tap = recall that key's face. Hold >= 3s
    // (any key but '*') = save prompt on j4_display_right; while the prompt
    // is up, '*' confirms and any other key cancels. Pure state tracking at
    // the same 150ms scan cadence -- nothing here blocks.
    bool rk_pressed = keypadReady(kpg_right) && kpIsPressed(kpg_right.pcf, kpg_right.pins);
    uint8_t rk_raw  = 16;
    if (rk_pressed) {
      rk_raw = kpGetKey(kpg_right.pcf, kpg_right.pins);
      if (rk_raw > 15) rk_pressed = false;   // ghost read -- treat as none
    }
    if (rk_pressed) {
      char kc = kpg_right.keymap[rk_raw];
      if ((int8_t)rk_raw != rk_down) {       // new key down
        rk_down       = (int8_t)rk_raw;
        rk_down_since = currentMillis;
        rk_hold_fired = false;
        rk_consumed   = false;
        last_key_char_right = kc;
        if (face_prompt_key) {               // this press answers the prompt
          rk_consumed = true;
          if (kc == '*') faceSaveConfirmed();
          else           facePromptCancel("CANCELLED");
        }
      } else if (!rk_hold_fired && !rk_consumed && !face_prompt_key
                 && currentMillis - rk_down_since >= FACE_HOLD_MS) {
        rk_hold_fired = true;
        if (kc != '*') facePromptOpen(kc);   // '*' is confirm-only, never a slot
      }
    } else if (rk_down >= 0) {               // key released
      if (!rk_hold_fired && !rk_consumed && !face_prompt_key
          && kpg_right.keymap[rk_down] != '*')
        faceRecall(kpg_right.keymap[rk_down]);
      rk_down = -1;
    }
  }

  // --- FACE PRESET BACKGROUND WORK (all timer-driven, never blocking) ---
  processFacePackets();

  if (face_prompt_key && currentMillis - face_prompt_started >= FACE_PROMPT_TIMEOUT_MS)
    facePromptCancel("TIMED OUT");

  if (faceMsg_clearAt && currentMillis >= faceMsg_clearAt) {
    faceMsg_clearAt = 0;
    Serial2.print("X:\n");
  }

  // Re-request the saved-face dump until a complete one lands. Fires only
  // while the whole talk chain is up; a robot with no talk board simply
  // never syncs and everything else keeps working.
  if (currentMillis - faceReq_previousMillis >= faceReq_interval) {
    faceReq_previousMillis = currentMillis;
    if (!faces_synced && talkLinkUp()) {
      faceDumpCount = 0;
      faceSendPkt(FACE_OP_REQ, NULL);
    }
  }

  // Push one dirty face per tick toward the SD; FACE_OP_ACK clears the flag.
  // Saves made while talk was offline sync themselves this way.
  if (currentMillis - faceDirty_previousMillis >= faceDirty_interval) {
    faceDirty_previousMillis = currentMillis;
    if (talkLinkUp()) {
      for (uint8_t i = 0; i < FACE_SLOTS; i++) {
        if (faces[i].valid && faces[i].dirty && !faces[i].sd_failed) {
          faceSendPkt(FACE_OP_SAVE, &faces[i]);
          break;
        }
      }
    }
  }
  // --- END FACE PRESET BACKGROUND WORK ---

}


// --- ESP-NOW RELATED ---
// Runs in the WiFi task: flag stores only. No String/heap work and no
// peripheral writes in here -- both raced loop() and corrupted the heap.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  connectError = (status != ESP_NOW_SEND_SUCCESS);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Only accept packets from our receiver -- at a crowded event another
  // ESP-NOW project's broadcast could otherwise be mistaken for ours.
  if (memcmp(mac, broadcastAddress, 6) != 0) return;
  if (len < 1) return;

  uint8_t pkt_type = incomingData[0];

  if (pkt_type == ESPNOW_PKT_FILELIST && len == sizeof(espnow_filelist_chunk_t)) {
    const espnow_filelist_chunk_t *chunk = (const espnow_filelist_chunk_t *)incomingData;

    if (chunk->chunk_index == 0) {
      // First chunk - reset and initialise the accumulator
      memset(&jukeboxPkt, 0, sizeof(jukeboxPkt));
      jukeboxPkt.magic[0] = 0xBE;
      jukeboxPkt.magic[1] = 0xCD;
      jukeboxPkt.total_files = 0;
      chunksReceived = 0;
      jukeboxReady   = false;
    }

    chunksExpected = chunk->total_chunks;

    for (uint8_t i = 0; i < chunk->entry_count && jukeboxPkt.total_files < MAX_FILES; i++) {
      uint8_t idx = jukeboxPkt.total_files;
      memcpy(jukeboxPkt.files[idx].id,   chunk->entries[i].id,   FILE_ID_LEN);
      memcpy(jukeboxPkt.files[idx].name, chunk->entries[i].name, FILE_NAME_MAX);
      jukeboxPkt.total_files++;
    }

    chunksReceived++;

    if (chunksReceived >= chunksExpected) {
      jukeboxReady = true;
      filelistForwardPending = true;   // forwarded from loop(), not this WiFi-task context
    }

  } else if (pkt_type == ESPNOW_PKT_STATUS && len == sizeof(struct_message_rcv)) {
    memcpy(&rcvData, incomingData, sizeof(rcvData));
    lastStatusRecvMs = millis();

  } else if (pkt_type == ESPNOW_PKT_FACE && len == sizeof(espnow_face_pkt_t)) {
    // memcpy-into-ring only (WiFi task context); loop() drains via
    // processFacePackets(). A full ring drops the packet -- the REQ/dirty
    // timers make every face exchange retryable, so nothing is lost for good.
    uint8_t next = (uint8_t)((faceRxHead + 1) % FACE_RXQ);
    if (next != faceRxTail) {
      memcpy((void *)&faceRxQ[faceRxHead], incomingData, sizeof(espnow_face_pkt_t));
      faceRxHead = next;
    }
  }
}
// --- END ESP-NOW RELATED ---


// --- XIAO LINK ---
void sendToXIAO() {
  disp_pkt_t pkt;
  pkt.magic[0]   = 0xAB;
  pkt.magic[1]   = 0xCD;
  // disp_pkt_t is kept byte-for-byte compatible with j4_display, so the new
  // controls reuse existing slots: eyes<-iris, spot<-eye_pop, left_arm<-eyes_x,
  // right_arm<-eyes_y.
  pkt.volume     = (uint8_t)volume_value;
  pkt.eyes       = (uint8_t)iris_value;
  pkt.spot       = (uint8_t)map(eye_pop_value, 0, 3200, 0, 255);
  // eyes_x/eyes_y are centre-zero here but these slots are uint8_t: casting a
  // negative value straight in would wrap it to the top of the range, so
  // re-centre to 0-255 the same way the ESP-NOW packet does.
  pkt.left_arm   = (uint8_t)constrain(eyes_x_value + 128, 0, 255);
  pkt.right_arm  = (uint8_t)constrain(eyes_y_value + 128, 0, 255);
  // neck-L/R are centre-zero here and these slots are uint8_t, so map from
  // the signed range: mapping from 0-3200 would send every negative value
  // through as a wrapped, very large byte.
  pkt.neck       = (uint8_t)constrain(map(neck_left_value,  -1600, 1600, 0, 255), 0, 255);
  pkt.jaw        = (uint8_t)constrain(map(neck_right_value, -1600, 1600, 0, 255), 0, 255);
  pkt.bat1_mv    = bat1_mv;
  pkt.bat2_raw   = rcvData.battery_02_voltage_rcv;
  pkt.bat3_raw   = rcvData.battery_03_voltage_rcv;
  pkt.connect_ok = connectError ? 0 : 1;
  strncpy(pkt.phrase, rcvData.phrase_playing_rcv, sizeof(pkt.phrase) - 1);
  pkt.phrase[sizeof(pkt.phrase) - 1] = '\0';

  // XOR checksum over all bytes except the final checksum byte
  uint8_t cs = 0;
  const uint8_t *p = (const uint8_t *)&pkt;
  for (size_t i = 0; i < sizeof(pkt) - 1; i++) cs ^= p[i];
  pkt.checksum = cs;

  Serial1.write((const uint8_t *)&pkt, sizeof(pkt));
}
// --- END XIAO LINK ---


void sendFileListToXIAO() {
  uint8_t cs = 0;
  const uint8_t *p = (const uint8_t *)&jukeboxPkt;
  for (size_t i = 0; i < sizeof(jukeboxPkt) - 1; i++) cs ^= p[i];
  jukeboxPkt.checksum = cs;
  Serial1.write((const uint8_t *)&jukeboxPkt, sizeof(jukeboxPkt));
}
// --- END XIAO LINK ---


void labelsDisplaySprite() {
  screen_bottom_sprite_203.fillRect(0, 20, 70, 160, TFT_BLACK);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("Playing: ",  0,  20, 2);
  screen_bottom_sprite_203.drawString("VOL: ",      0,  40, 2);
  screen_bottom_sprite_203.drawString("Keypad_R: ", 0,  60, 2);
  screen_bottom_sprite_203.drawString("Eye-X: ",    0,  80, 2);
  screen_bottom_sprite_203.drawString("Eye-Y: ",    0, 100, 2);
  screen_bottom_sprite_203.drawString("Neck-L: ",   0, 120, 2);
  screen_bottom_sprite_203.drawString("Neck-R: ",   0, 140, 2);
  screen_bottom_sprite_203.drawString("Neck-PIV: ", 0, 160, 2);
}


// Combined system status: OFFLINE if the receiver link is silent, otherwise
// ONLINE, or whatever fault the stepper chain reported (e.g. "NL OT").
String buildStatusLine() {
  if (millis() - lastStatusRecvMs > STATUS_LINK_TIMEOUT_MS) return "OFFLINE";
  String s = rcvData.stepper_status_rcv;
  if (s.length() == 0 || s == "OK") return "ONLINE";
  return s;
}


void dataDisplaySprite() {
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.fillRect(70, 40, 65, 140, TFT_BLACK);

  if (!ready_message) {
    screen_bottom_sprite_203.drawString("Keypad_L: ",        0,  0, 2);
    screen_bottom_sprite_203.drawString(String(last_key_char), 70, 0, 2);
  }

  screen_bottom_sprite_203.drawString(String(volume_value),          70,  40, 2);
  screen_bottom_sprite_203.drawString(String(last_key_char_right),   70,  60, 2);
  screen_bottom_sprite_203.drawString(String(eyes_x_value),          70,  80, 2);
  screen_bottom_sprite_203.drawString(String(eyes_y_value),          70, 100, 2);
  screen_bottom_sprite_203.drawString(String(neck_left_value),       70, 120, 2);
  screen_bottom_sprite_203.drawString(String(neck_right_value),      70, 140, 2);
  screen_bottom_sprite_203.drawString(String(neck_pivot_value),      70, 160, 2);

  // System status line (green when healthy, red on any fault / link loss)
  String st = buildStatusLine();
  bool ok = (st == "ONLINE");
  screen_bottom_sprite_203.fillRect(0, 182, 135, 18, TFT_BLACK);
  screen_bottom_sprite_203.setTextColor(ok ? TFT_GREEN : TFT_RED);
  screen_bottom_sprite_203.drawString("STATUS: " + st, 0, 182, 1);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
}


void tftDisplayUpdate() {
  if (screen_mode == 0) {
    dataDisplaySprite();
    screen_bottom_sprite_203.pushSprite(0, 38);
  } else if (screen_mode == 1) {
    macAddressDisplay();
  } else if (screen_mode == 2) {
    connectionDisplay();
  } else {
    adsPotsDisplay(screen_mode - 3);   // 3-7: ADS_01..ADS_04, then DISP_R
  }
}


// Cycle screens with the TTGO's built-in GPIO 35 button:
// data -> MAC -> status -> ADS_01..ADS_04 pots -> j4_display_right's ADS1115.
void controllerScreenModeDetect() {
  if (millis() - screen_button_previousMillis >= screen_debounce_ms) {
    bool cur = digitalRead(SCREEN_BUTTON);
    if (screen_button_prev == HIGH && cur == LOW) {
      screen_mode = (screen_mode + 1) % NUM_SCREENS;
      if (screen_mode == 0) {
        tft.fillScreen(TFT_BLACK);
        tft.pushImage(0, 0, 135, 37, kevco_labs_logo_02);
        labelsDisplaySprite();
      } else if (screen_mode == 1) {
        macAddressDisplay();
      } else {
        tft.fillScreen(TFT_BLACK);   // clear once; connectionDisplay()/adsPotsDisplay() repaint values
      }
      screen_button_previousMillis = millis();
    }
    screen_button_prev = cur;
  }
}


void macAddressDisplay() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("WiFi MAC:", 10, 60, 2);
  tft.drawString(WiFi.macAddress(), 10, 90, 2);
}


// One "<name>: CONNECTED/DISCONNECTED" row (green/red), label and state stacked.
void drawConnLine(const char *name, bool ok, int row) {
  int y = 10 + row * 40;
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(name, 6, y, 2);
  tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(ok ? "CONNECTED   " : "DISCONNECTED", 6, y + 17, 2);
}


// Connection-status screen: how this controller sees each link right now.
void connectionDisplay() {
  unsigned long now = millis();
  bool espnow  = (now - lastStatusRecvMs) < STATUS_LINK_TIMEOUT_MS;   // receiver link
  bool dispLOk = (now - lastDisplayMs)    < DISPLAY_TIMEOUT_MS;
  bool dispROk = (now - lastDisplayRMs)   < DISPLAY_TIMEOUT_MS;
  bool neckOk  = espnow && rcvData.stepper_ok_rcv;                    // relayed by receiver
  bool eyesOk  = espnow && rcvData.eyes_ok_rcv;
  bool talkOk  = espnow && rcvData.talk_ok_rcv;
  drawConnLine("ESP-NOW LINK",     espnow,  0);
  drawConnLine("j4_stepper_neck",  neckOk,  1);
  drawConnLine("j4_stepper_eyes",  eyesOk,  2);
  drawConnLine("j4_talk",          talkOk,  3);
  drawConnLine("j4_display_left",  dispLOk, 4);
  drawConnLine("j4_display_right", dispROk, 5);
}


// Live raw counts off one ADS1115 module (screen_mode 3-6), for bench
// testing without needing a laptop on the I2C bus. Module presence reuses
// the same adsReady() guard the control-tx loop already probes with.
void adsPotsDisplay(int idx) {
  AdsPotScreen &s = adsPotScreens[idx];
  // Local modules ACK on I2C; the remote one is "present" while its serial
  // feed is fresh.
  bool ok = s.guard ? adsReady(*s.guard)
                    : (millis() - lastDisplayRMs < DISPLAY_TIMEOUT_MS);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(s.title, 6, 4, 2);
  tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(ok ? "CONNECTED   " : "DISCONNECTED", 6, 22, 2);

  for (int i = 0; i < 4; i++) {
    int y = 46 + i * 40;

    // Line 1: channel name and its raw ADS1115 count.
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(s.labels[i], 0, y, 2);
    tft.fillRect(70, y, 65, 17, TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    int raw = ads_raw[s.module][i];
    tft.drawString(raw < 0 ? "--" : String(raw), 70, y, 2);

    // Line 2: what that channel drives, and the value that leaves this board.
    int y2 = y + 18;
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString(s.dests[i], 8, y2, 2);
    tft.fillRect(70, y2, 65, 17, TFT_BLACK);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(s.values[i] ? String(*s.values[i]) : "--", 70, y2, 2);
  }
}
