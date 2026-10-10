# Project History

## Student Aerospace Challenge 2025/2026

INOAS began as the Supaero Astra Iberian Team project for WP7: Reusable
Propulsion / Maintenance of the
[Student Aerospace Challenge](https://www.studentaerospacechallenge.eu/index.php/en).
The team presented it at Aerospace Challenge Day, Paris-Le Bourget, on
June 25, 2026.

The Challenge demonstrator used a 10 t spacecraft and different encounter
and navigation settings. Development subsequently continued with a CubeSat
configuration. The material below documents the Challenge project; consult
[model architecture](model-architecture.md) for the active implementation.

## Challenge Architecture

![Original Student Aerospace Challenge architecture](assets/history/challenge-architecture.png)

This original diagram shows the Challenge design, including its earlier
navigation selector and receiver-supervision concept. It is not a literal
block diagram of the current model: current guidance always uses the UKF
estimate, GNSS observations are emulated with processed profiles, and receiver
power is distinct from correction enable.

## Presentation Material

[Challenge poster PDF](assets/history/challenge-poster.pdf) |
[Full-quality presentation PPTX](https://github.com/enriquev212/INOAS/releases/download/inoas-project-materials-v1/INOAS_full_quality_final_presentation.pptx)

![Student Aerospace Challenge poster](assets/history/challenge-poster.png)

The poster and presentation are retained in their original form. Their
figures and performance statements concern the Challenge configuration only.

## Preserved Playback

![Challenge debris-avoidance playback](assets/debris-avoidance-playback.gif)

This animation is retained unchanged from the earlier demonstrator, not
regenerated from the current default CubeSat configuration.

Earlier development states are preserved in a
[maintainer-held archive](model-provenance.md#archived-development-branches),
not as alternative supported defaults. The presentation remains available in
the [original project-materials Release](https://github.com/enriquev212/INOAS/releases/tag/inoas-project-materials-v1).
