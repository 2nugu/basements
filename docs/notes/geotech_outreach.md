# Geotech Outreach — Tier 1→4 Cascade

**Purpose:** Concrete decision tree for procuring measured sand
parameters ($E$, $\nu$, $\phi$, $\rho_{\text{bulk}}$, $d_{50}$) that
back the paper §3 calibration claim. Without measured parameters,
paper §3 is reduced to "literature values" — a weak claim and easy
reviewer attack vector.

**Hard deadline:** end of W2 to start outreach; W3 mid for parameter
data in hand; W3 end for sim re-calibration with measured values.
If timing slips, fall back per the cascade.

---

## Tier 1 — Strongly preferred

**Action:** Outreach to Kangwon National University Geotechnical
Engineering laboratory (user's institution).

**Effort:** 1 email + 1–2 follow-up meetings.

**Output:**
- Measured ($E$, $\nu$, $\phi$, $\rho$, $d_{50}$) with ASTM-standard
  test certificate
- Lab name cited in paper §3.2 — "Measured at [Kangwon NU Geotech
  Laboratory], ASTM D-3080 + D-2435 + D-2434 + D-422"
- Possible co-authorship for substantial measurement contribution

**Cost:** $0 + 1 kg sand sample ($5).

**Timeline:** 1–2 weeks (depends on lab availability).

**Email template:**

```
제목: 사질토 시뮬레이션 검증을 위한 표준 시험 협력 문의

[교수님 성함] 교수님께,

안녕하세요. [작성자]입니다. 현재 입자체-강체 결합 시뮬레이션
(MPM 기반)을 개발 중이며, 시뮬레이터의 정확도 검증을 위해 사용
중인 사질토 시료의 E (Young's modulus), ν (Poisson ratio),
φ (friction angle), ρ_bulk, d_50 측정값이 필요합니다.

문헌값(Klar 2016, Wong 1989 등) 대신 실제 측정값을 paper §3에
명시하면 sim의 외부 검증 가능성이 한 단계 높아집니다.

가능하시다면 1 kg 표준 시료(Ottawa F-65 또는 주문진 모래)에 대해
다음 시험 협력을 부탁드립니다:
  - 직접전단시험 (φ) — ASTM D-3080
  - 압밀시험 (E) — ASTM D-2435
  - 단위중량 (ρ) — ASTM D-2434
  - 입도분석 (d_50) — ASTM D-422

연구실 일정 가능하시면 1회 협의 미팅 요청드리며, 결과는 SIGGRAPH
또는 IEEE-RAS 논문에 공동저자 또는 acknowledgment로 명시 가능합니다.

답변 부탁드립니다.

[작성자]
[연락처]
```

**Risk:** Professor declines or unresponsive. → Tier 2.

---

## Tier 2 — Paid service fallback

**Action:** Commission KICT (Korea Institute of Civil Engineering and
Building Technology) or Korean Geotechnical Society affiliated testing
laboratory for the same suite of tests.

**Output:** Same as Tier 1 minus co-authorship. Paper §3.2 cites the
testing institution.

**Cost:** ~₩1,000,000 ($750) for the full suite (5 tests × ~₩200k each).

**Timeline:** 2–4 weeks (queue + processing).

**Decision criterion:** Trigger this if Tier 1 has not responded by
W2 end. Sample is shipped immediately on trigger.

**Contact starting points:**
- KICT: https://www.kict.re.kr/
- 한국지반공학회: http://www.kgshome.or.kr/

---

## Tier 3 — Student-membership service

**Action:** Join Korean Geotechnical Society as student member
(₩50k/yr), then commission tests at member rates.

**Output:** Same as Tier 2 with slight cost reduction.

**Cost:** ₩50k (membership) + ~₩400k (member-rate tests).

**Timeline:** 2–3 weeks.

**Decision criterion:** Trigger if both Tier 1 fail AND Tier 2
queue exceeds 3 weeks.

---

## Tier 4 — DIY portable shear vane (last resort)

**Action:** Procure portable shear vane apparatus and measure $\phi$
self-administered. $E$, $\nu$ would still need literature values.

**Output:** "Author-measured $\phi$ with calibrated portable shear
vane. $E$, $\nu$ adopted from [Klar 2016] for the same sand type."

**Cost:** ~$500 (shear vane); time ~1 week.

**Decision criterion:** Last resort if all paid options unavailable.

**Risk:** Weaker paper claim — reviewer may attack "author-measured"
as non-ASTM-standard. Mitigation: report inter-trial repeatability
explicitly + cite literature comparison.

---

## Decision flowchart

```
W2 start ── send Tier 1 email
            │
            ├── reply within 1 week → schedule meeting
            │                          │
            │                          ├── lab agrees → proceed Tier 1
            │                          └── lab declines → Tier 2
            │
            └── no reply by W2 end → send Tier 2 commission
                                     │
                                     ├── queue < 3 weeks → proceed Tier 2
                                     └── queue ≥ 3 weeks → Tier 3
                                                            │
                                                            ├── available → Tier 3
                                                            └── unavailable → Tier 4

W3 mid → parameters in hand → update config/sandbox_USER.yaml
W3 end → sim re-calibrated with measured values
W4 → experiment proceeds with measured parameters
```

---

## Status

- [ ] Tier 1 — Kangwon NU professor identified: ________________
- [ ] Tier 1 — email sent: ________________ (date)
- [ ] Tier 1 — response received: ________________ (date)
- [ ] Tier 2 — KICT commission filed (if needed): ________________
- [ ] Parameter data received: ________________ (date)
- [ ] sandbox_USER.yaml populated with measured values

---

## Change log

- 2026-05-15 — Initial cascade design. Hard W2 deadline for outreach
  trigger; W3 mid for data; W4 for experiment.
