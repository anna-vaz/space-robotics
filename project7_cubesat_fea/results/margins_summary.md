# Project 7 — margins of safety, design v5 (4.000 kg)

Factors of safety: 1.25 yield, 1.4 ultimate (NASA-STD-5001B, analysis only); random vibration design stress = 3σ.
Allowables: MMPDS A-basis 6061-T6 Fty 241 / Ftu 290 MPa, 7075-T6 Fty 462 / Ftu 524 MPa. MS = allowable / (FS × stress) − 1.

| Load case | Requirement | Governing item | Result | Margin (yield / ult) | Independent check |
|---|---|---|---|---|---|
| Modal, cantilever (SOLIDWORKS) | f1 > 100 Hz | board-stack local mode | 225 Hz; first global bending 216–235 Hz | f1 / 100 Hz = 2.25 | Ansys 216 Hz Y bending (−8 %); Timoshenko hand calc 255 Hz |
| 10 g quasi-static X (Ansys) | FS 1.25 / 1.4 | end plate -Z | 15.0 MPa | +11.89 / +12.85 | SOLIDWORKS 14.4 MPa; reaction 392.26 N = m·10 g |
| 10 g quasi-static Y (Ansys) | FS 1.25 / 1.4 | spacer 13 mm @ (7.135, 9.995) z 115-128 | 14.1 MPa | +12.71 / +13.73 | SOLIDWORKS 15.2 MPa; reaction 392.26 N = m·10 g |
| 10 g quasi-static Z (Ansys) | FS 1.25 / 1.4 | RW bulkhead | 6.0 MPa | +30.92 / +33.30 | SOLIDWORKS 6.1 MPa; reaction 392.26 N = m·10 g |
| Buckling at 10 g, X (SOLIDWORKS v5, no panels) | BLF ≥ 10.6 | first mode | BLF 116.9 | +10.03 | hand (step 1): rails alone between 1U frames +0.78 (lower bound) |
| Buckling at 10 g, Y (SOLIDWORKS v5, no panels) | BLF ≥ 10.6 | first mode | BLF 48.7 | +3.59 | hand (step 1): rails alone between 1U frames +0.78 (lower bound) |
| Buckling at 10 g, Z (SOLIDWORKS v5, no panels) | BLF ≥ 15.7 | first mode | BLF 116.0 | +6.39 | hand (step 1): rails alone between 1U frames +0.78 (lower bound) |
| Random vibration GEVS qual X (Ansys PSD) | FS 1.25 / 1.4 on 3σ | spacer 13 mm @ (7.135, 9.995) z 115-128 | 106.7 MPa 3σ | +0.81 / +0.94 |  |
| Random vibration GEVS qual Y (Ansys PSD) | FS 1.25 / 1.4 on 3σ | spacer 13 mm @ (7.135, 9.995) z 115-128 | 121.0 MPa 3σ | +0.59 / +0.71 | Miles 29.3 g vs FE 28.6 g (1σ) |
| Random vibration GEVS qual Z (Ansys PSD) | FS 1.25 / 1.4 on 3σ | RW bulkhead | 51.7 MPa 3σ | +2.73 / +3.00 |  |
| M3 screws, all spacer ends, 3 axes (NASA-STD-5020A) | separation 1.2, yield 1.25, ult 1.4 | 13 mm spacer, bottom, Y | Pt 623 N | sep +0.10, yield +0.09, ult +2.95 | hand: Pt = F + M/r |
| Light case 3.565 kg, random vibration Y | as above | 13 mm spacer | 110.1 MPa 3σ | +0.75 / +0.88 | screws sep +0.21, yield +0.19; f1 +7.2 % |

Every margin is positive. The tightest are the M3 screw preload window (+0.09 yield, +0.10 separation) and the
spacers above the 1U rib in lateral random vibration (+0.59; about +0.49 with the tapped bore). Static 10 g is not a
sizing case (margins > +11). The 4.000 kg design case governs; the light case has higher margins everywhere.
