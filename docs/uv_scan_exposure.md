# UV scan exposure: medium, LEDs, distance, speed

Research notes for the LED-strip scan (2026-09-28): what the "UV paint" is
likely to be, which LEDs expose it, how close the strip should pass and how
fast. Sources are linked; estimates are marked as ours. Settle the numbers
with a test strip on the real paint.

## The medium

| Medium | Exposed by | Fits "a second pass double-exposes, one fast pass"? |
|---|---|---|
| Phosphorescent (strontium aluminate, ZnS) | 200-450 nm, best 365-405 nm; glows minutes to hours ([Wikipedia](https://en.wikipedia.org/wiki/Strontium_aluminate), [Wildfire](https://store.wildfirelighting.com/paints/wildfire-phosphorescent-paints/phosphorescent-paint/)) | Yes: low doses leave a visible trace |
| Photochromic | 300-360 nm UV; darkens, fades back ([SpotSee](https://spotsee.io/technologies/photochromic/)) | Possibly |
| Cyanotype | 365 nm about twice as fast as 395 nm ([Waveform](https://www.waveformlighting.com/knowledgebase/d50/uv-led-light-for-cyanotype-printing-application)); first visible at ~3.4 mJ/cm^2 ([paper](https://www.researchgate.net/publication/266630860_From_Ultraviolet_to_Prussian_blue_A_spectral_response_for_the_cyanotype_process_and_a_safe_educational_activity_to_explain_UV_exposure_for_all_ages)); a full print 3-8 min of summer sun ([Cyanoprints](https://www.cyanoprints.com/blogs/news/cyanotype-belichtingstijd-tips)), ~0.5-3 J/cm^2 | No: minutes per line |
| UV-curing ink (395 nm) | 100-440 mJ/cm^2 ([WO2022106853A1](https://patents.google.com/patent/WO2022106853A1/en)) | No |
| Fluorescent "blacklight" paint | glows only while lit, nothing stored | Not exposable |

Most likely (ours): phosphorescent paint. The closest precedent is
[Glowxels](https://github.com/hzeller/glowxels) (a bar of 405 nm LEDs at
1.5 mm pitch writing onto a glow canvas); also
[Glowtape](https://github.com/hzeller/glowtape),
[Ghost Matrix](https://hackaday.com/2009/05/23/ghost-matrix-glow-printing/),
[UV glow laser displays](https://hackaday.io/project/179950-uv-glow-in-the-dark-laser-displays).
Robot-arm light painting: [CMU / ABB](https://creators.vice.com/en_us/article/yp5pvm/three-dimensional-light-paintings-made-by-a-giant-robot-arm),
[Noelle and Braumann / KUKA](https://ars.electronica.art/aeblog/en/2016/02/02/lightpainting/).

## The LEDs

Ordinary WS2812 RGB strips emit next to nothing below ~450 nm. Individually
addressable UV strips exist: GS8208 at 395 nm, 60/m, 12 V, 14.4 W/m
([Suntech](https://suntechlite.com/addressable-uv-led-strip-individually-controlled/));
WS2811 UV strips switch LEDs in groups of 3-6. A 395 nm 5050 LED gives
~45-50 mW of UV at 50 mA ([LEDWV](https://www.ledwv.com/uv/uv-leds-c-19/uv-led-smd-5050-395nm-p-757.html)).

### The strip bought: Adafruit 5722

[Adafruit High Density NeoPixel UV LED Strip, 60 LED/m, 1 m](https://www.adafruit.com/product/5722)
([LED datasheet](https://cdn-shop.adafruit.com/product-files/5722/5722_datasheet.pdf)):

- 60 UV LEDs per metre (pitch 16.7 mm), each a 5050 package of three InGaN dies
  on a WS2811 driver: individually addressable, NeoPixel protocol; R, G and B
  drive the three dies -- set all three alike for 0-255 brightness.
- **395 nm** dominant (390-400), viewing angle 2 theta = 120 deg; 20 mA a die,
  60 mA an LED: ~3.6 A a metre at full brightness, **5 V only** (over 6 V
  destroys the strip). Weatherproof sheathing, a 2-pin JST SM at each end, cut
  lines every LED.
- Radiant (UV) power is not given (mcd means little at 395 nm); ~50 mW an LED
  at full current is typical of the class, ~3 W/m -- the figure used below.

For the rig (ours): 3.6 A at 5 V through the arm's cable wants a thick pair
(or the supply at the tool) and power fed at both ends; the 800 kHz data line
over the arm's few metres wants a 5 V level shifter at the controller and
perhaps a differential link; the cable's J6 range is +-150 deg
(`profiles/fr20.json` tool.cable_j6_deg). 395 nm charges strontium aluminate
well; it is weak for cyanotype.

**A strip that fits between the rails**: cut at 53 LEDs it is ~0.885 m, under
the frame's 0.902 m opening, and can pass ~1 cm from the paper (sharp lines,
neighbours blend at ~1 x pitch) -- but then the scan cannot run up over the
side rails either: it would come straight in at the opening's left edge and
leave straight out at the right, its ramps over the paper, with the LEDs'
brightness following the speed (dose = P/v kept even: a laser cutter's
"dynamic power", GRBL's M4 mode). The whole 1 m strip passes over the rails
at 6 cm (now).

## Distance and dose (ours)

For a line of wide-angle (~120 deg) LEDs with UV power P_L per metre at a
distance d from the paper:

- across the scan the light is ~P_L / (2 d) at its peak and ~1.5 d + the LED's
  own size (~5 mm) wide at half power -- the blur of one line;
- **dose per pass = P_L / v** whatever d is; d changes the sharpness and the
  peak, not the dose;
- the pixels along the strip blend when d is ~0.6-1 x the LED pitch
  (~10-17 mm at 60/m, 5-7 mm at 144/m).

At ~3 W/m of UV (60/m x 50 mW): 0.1 m/s gives ~3 mJ/cm^2 a pass, 0.7 m/s
~0.4 mJ/cm^2. A cyanotype's ~1 J/cm^2 would need ~0.3 mm/s.

**This frame**: its 5.25 in rails overlap the paper and stand ~2 cm in front
of it, and the opening (0.90 m high) is shorter than the 1 m strip, so the
strip passes over the rails. With the scan's 3 cm margin (collision, from the
strip's containing capsule) the LED face is set 6 cm from the paper
(`scan.led_gap_m`): a line blurred over ~9-10 cm. Closer needs either a
strip shorter than the opening (it then passes between the rails, ~1 cm), or
a black shroud with a 5-10 mm slit to cut the light's tails, and the frame's
real depth measured.

## Speed

Test 0.05-0.3 m/s on the real paint. The scan is built at `scan.speed_mps`
(0.2 m/s: the fastest this frame fits in the work zone -- 0.3 needs ~6 cm more
at the right end) with its ramps over the rails, so the opening is crossed at
an even speed; `show_stream.py --scan-speed f` (the window's Scan speed)
plays the scan alone at f of that (0.1-1) for tuning. The scan's labels give
`led_on_s`: when the strip is over the opening -- light it only then.

## Safety

The ICNIRP UV-A eye limit is 1 J/cm^2 for exposures under 1000 s
([ICNIRP](https://www.icnirp.org/cms/upload/publications/ICNIRPled2020.pdf));
400-500 nm adds a blue-light retinal hazard
([IEC 62471](https://www.signliteled.com/iec-62471-blue-light-hazard-explained/)).
365 nm looks dim, so people do not look away; 395/405 nm looks violet. For a
party: shroud the strip so it faces only the paper, light it only over the
paper, keep the audience back.
