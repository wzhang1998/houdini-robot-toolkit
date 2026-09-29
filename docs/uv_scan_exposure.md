# UV scan exposure: photochromic printing with the LED strip

Research notes for the LED-strip scan (2026-09-28): the reference work, the
medium, the strip bought, distance, dose and speed. Sources are linked;
estimates are marked as ours. Settle the numbers with a swatch test on the
real coating (at the end).

## The reference: Random International

**Presence and Erasure** (2019, [studio page](https://www.random-international.com/presence-and-erasure)):
"photochromic varnish, ply boards, linear rails, anodised aluminium, motor,
LEDs, phones, custom hardware and software", 800 x 240 x 30 cm. Faces seen by
its cameras are printed "by exposing a photochromic surface to light
impulses"; each portrait remains "little more than a minute, before gradually
dissolving into blankness". The studio calls it the latest step in
"transient mark-making with automated portraiture", a line begun in 2008.

Earlier works on light-reactive surfaces:
[Temporary (Light) Printing Machine](https://vernissage.tv/2007/01/05/random-international-temporary-light-printing-machine-scope-miami-2006/)
(2006; "custom rail system, light reactive screen print on canvas, motor,
electronic UV glass LED print head" per a Carpenters Workshop listing, now
offline; images visible "for about one minute"),
[Study for a Mirror](https://collections.vam.ac.uk/item/O1225416) (2009-10,
V&A: "light reactive screen print back-printed on glass with UV coating";
software by [Chris O'Shea](https://www.chrisoshea.org/portfolio/study-for-a-mirror)),
[Self Portrait](https://www.designboom.com/design/random-international-at-design-miami-basel-2010/) (2010).
Not photochromic: Temporary Graffiti (glow vinyl), Audience, Mirrors,
Fragments (mirrors), You Fade to Light, Study of You (OLED).
No published source gives their wavelength, LED pitch, distance, speed or
varnish.

Our rig is the same idea with a robot arm for the rail: a vertical UV LED
head swept once across a photochromic canvas, the image written column by
column and fading back in about a minute.

## The medium: photochromic

- Photochromic inks (spiropyran / spirooxazine / naphthopyran) colour under UV
  and fade back in the dark. [LCR Hallcrest / SpotSee](https://spotsee.io/technologies/photochromic/):
  activated by 300-360 nm, "intensely colored after only 15 seconds in direct
  sunshine", clear again in ~5 min indoors; fades faster when warm; "thousands"
  of cycles, UV slowly wears it; water-based, brush / roller / screen, best on
  an absorbent white ground, can be over-varnished
  ([art kit guide](https://spotsee.io/wp-content/uploads/2021/05/SM143-Rev01-Photochromic-Art-Kit-Users-Guide-1.pdf),
  [FAQ](https://spotsee.io/photochromic-ink-faqs/)).
  [QCR Solutions](https://qcrsolutions.com/photochromic-2/): dyes,
  encapsulated pigments, inks -- results depend on the UV source, binder and
  loading.
- At our wavelength: [PMC9084306](https://pmc.ncbi.nlm.nih.gov/articles/PMC9084306/)
  coloured a Reversacol Ruby Red ink with a 395-410 nm LED at 8 mW/cm^2:
  colouring time constant ~6 s, **fade half-life ~63 s** -- Presence and
  Erasure's "little over a minute".
- Midday sun gives ~2-6 mW/cm^2 of UV-A (320-400 nm).
- Coating (ours, from the above): encapsulated pigment in a clear water-based
  acrylic over a white gesso ground; **no UV-blocking top coat** (most
  "UV-protect" clears stop the activation). Blue / purple (spirooxazine) for
  contrast. Visible and white light bleach the coloured form a little;
  daylight or fluorescent UV tints the whole canvas: a dim room without UV.

### The pigment bought

[Sun UV activated photochromic powder pigment, white to violet](https://www.amazon.com/dp/B0747XQ3X9)
(sold for slime, nails and resin; the listing gives no wavelength or fade
time). A powder: it needs a clear binder. Ours, from the notes above: a few
per cent by weight in a clear water-based acrylic (matte / gel medium) over a
white gesso ground, no UV-blocking top coat; violet on white is the contrast.
Mix swatches at ~2, 5 and 10 % and let the bench test pick the loading
(saturation, evenness, and how fast it fades back).

## The strip bought: Adafruit 5722

[Adafruit High Density NeoPixel UV LED Strip, 60 LED/m, 1 m](https://www.adafruit.com/product/5722)
([LED datasheet](https://cdn-shop.adafruit.com/product-files/5722/5722_datasheet.pdf)):

- 60 UV LEDs per metre (pitch 16.7 mm), each a 5050 package of three InGaN dies
  on a WS2811 driver: individually addressable, NeoPixel protocol; R, G and B
  drive the three dies -- set all three alike for 0-255 brightness.
- **395 nm** dominant (390-400), viewing angle 2 theta = 120 deg; 20 mA a die,
  60 mA an LED: ~3.6 A a metre at full brightness, **5 V only** (over 6 V
  destroys the strip). Weatherproof sheathing, a 2-pin JST SM at each end, cut
  lines every LED.
- Radiant (UV) power is not given; ~50 mW an LED at full current is typical of
  the class, ~3 W/m -- the figure used below.
- 395 nm sits on the tail of most photochromics' absorption (quoted 300-360
  nm): it works (the Ruby Red test above) but needs several times the dose of
  sunlight's shorter UV.

For the rig (ours): 3.6 A at 5 V through the arm's cable wants a thick pair
(or the supply at the tool) and power fed at both ends; the 800 kHz data line
over the arm's few metres wants a 5 V level shifter and perhaps a differential
link; the cable's J6 range is +-150 deg (`profiles/fr20.json`
tool.cable_j6_deg).

## Distance and dose (ours)

For a line of wide-angle (~120 deg) LEDs with UV power P_L per metre at a
distance d from the canvas:

- **dose per pass H = P_L / v**, whatever d is; d sets the peak irradiance
  (~P_L / band) and the sharpness: the band across the scan is ~1.5 d + 5 mm
  wide at half power;
- the dwell (band / v) is the time a point is lit -- the same as a static
  exposure at that irradiance;
- along the strip the LEDs blend when d is at least ~1 x the pitch (16.7 mm):
  closer than ~2 cm the rows turn spotty.

At P_L ~ 3 W/m:

| v (m/s) | H a pass (mJ/cm^2) | dwell at 3 cm (5 cm band) | at 6 cm (9.5 cm band) | a 1.44 m scan |
|---|---|---|---|---|
| 0.20 | 1.5 | 0.25 s | 0.5 s | 7 s |
| 0.10 | 3 | 0.5 s | 1 s | 14 s |
| 0.05 | 6 | 1 s | 1.9 s | 29 s |
| 0.02 | 15 | 2.5 s | 4.8 s | 72 s |

Peak irradiance ~6 mW/cm^2 at 3 cm (about one sun of UV-A), ~3 at 6 cm.
Near-full colour of the Ruby Red ink at 395-410 nm takes ~150 mJ/cm^2: one
pass gives ~1-10 % of it at 0.2-0.05 m/s and ~20-25 % at 0.02 m/s. **The
rig is dose-limited**: write slowly, in a dim room, on a bright white ground,
with a pigment that answers 395 nm well -- or add a second strip beside the
first (twice P_L).

**This frame**: its 5.25 in rails overlap the canvas and stand ~2 cm in front
of it (to measure), and its opening (0.90 m high) is shorter than the 1 m
strip, so the whole strip passes over the rails. The build keeps the strip's
collision capsule `margins.scan_canvas_m` (3 cm) clear of them, which puts
the LED face 6 cm from the canvas (`scan.led_gap_m`). ~3 cm is the sweet
spot (even light along the strip, a 5 cm band, twice the irradiance); it
needs the rails' depth measured and a smaller margin over them, or a strip
cut shorter than the opening (53 LEDs ~0.885 m) passing between the rails
-- its ramps then over the canvas with the brightness following the speed
(dose = P/v kept even: a laser cutter's "dynamic power", GRBL's M4 mode).

## Speed and the fade

- Write at **0.02-0.05 m/s** (72-29 s a scan); 0.1-0.2 m/s is a travel speed,
  not a writing speed. The scan is built at `scan.speed_mps` (0.2 m/s: the
  fastest this frame fits the work zone), crossing the opening at an even
  speed with its ramps over the rails; `show_stream.py --scan-speed f` (the
  window's Scan speed, 0.05-1) plays the scan alone at f of that -- 0.1 is
  0.02 m/s. The scan's labels give `led_on_s`, when the strip is over the
  opening (at the built speed; divide by f).
- With a ~60 s half-life the first column fades while the last is written: a
  29 s scan leaves the first edge at ~70 % of the last, a 72 s one at ~45 %.
  Compensate in the LED brightness (a column written t after the first lit
  2^(-t/t_half) of full: the later columns dimmer, so all have faded to the
  same by the end of the pass; TD-ROBOT-UVSCAN pixel_scan's Fade
  Compensation), or accept it as part of the piece. Warm rooms fade faster (roughly 2-3x per
  +10 C): keep the canvas cool.
- A second pass while the image is still coloured adds dose (darker, not
  over-exposed) -- the user's choice is one pass, left to right, so the scan
  never goes back over the canvas.

## Swatch test before the show

A coated swatch under the strip at 3 cm and at 6 cm, lit statically for 0.5,
1, 2.5, 5, 10 and 20 s; photograph the fade every 10 s; compare with a 365 nm
torch. That gives the dose curve and the half-life of this coating at 395 nm,
and so the scan speed. With the white-to-violet powder: one swatch per
loading (2, 5, 10 %), each strip of it lit for a different time -- the step
where the violet stops deepening is the dose to aim for; the dwell of a scan
point is (1.5 d + 5 mm) / v.

## Safety

ICNIRP: 10 kJ/m^2 of unweighted 315-400 nm a day for eyes and skin
([ICNIRP 2004](https://www.icnirp.org/cms/upload/publications/ICNIRPUV2004.pdf));
400-500 nm adds a blue-light retinal hazard
([IEC 62471](https://www.signliteled.com/iec-62471-blue-light-hazard-explained/)).
The strip faces the canvas, so guests see scattered light (~0.5 W/m^2 at 1 m:
hours to the daily limit, ours). Do not let anyone stare into it close up; a
black flocked hood or skirt round the strip cuts the spill, the stray tint on
the rails and the glare. Light it only over the canvas.
