# Reading list

Literature behind the traversability, visual-navigation and planning choices in
this project, in the order it is worth reading.

The PDFs themselves are deliberately **not tracked by git** (`papers/*.pdf` is in
`.gitignore`); this file is the list. Keep your local copies in this folder under
the numbering below. Items 3, 6 and 8 were fetched; the rest are listed with their
links because the publisher sites refuse scripted downloads or the paper is
paywalled.

| # | Paper | Venue | Access | Fetched |
|---|---|---|---|---|
| 1 | *Overview of Terrain Traversability Evaluation for Autonomous Robots* | J. Field Robotics, 2024 (doi:10.1002/rob.22461) | Paywalled; no open copy found | No |
| 2 | *Advances and Trends in Terrain Classification Methods for Off-Road Perception* | J. Field Robotics, 2025 (doi:10.1002/rob.22586) | Open access, CC BY-ND | No (site blocks scripted download) |
| 3 | *Wild Visual Navigation: Fast Traversability Learning via Pre-Trained Models and Online Self-Supervision* | Autonomous Robots 49:19, 2025 (doi:10.1007/s10514-025-10202-x) | Open access, CC BY | **Yes** |
| 4 | *Toward GPS-independent ground vehicle control: visual navigation and traversability-aware planning in unstructured environments* | Defence Technology, 2026 (doi:10.1016/j.dt.2026.04.016) | Open access, CC BY-NC-ND | No (site blocks scripted download) |
| 5 | *A Global Path Planning Method for Unmanned Ground Vehicles in Off-Road Environments Based on Mobility Prediction* | Machines 10(5):375, 2022 (doi:10.3390/machines10050375) | Open access, CC BY | No (site blocks scripted download) |
| 6 | Nav2 MPPI controller configuration guide | Nav2 documentation (rolling) | Public web page | **Yes**, as a PDF snapshot taken 2026-10-06 |
| 7 | *One-stage motion planning with probabilistic traversability risk awareness for unmanned ground vehicles in unstructured environments via accelerated model predictive path integral control* | Green Energy and Intelligent Transportation, July 2026 (doi:10.1016/j.geits.2026.100444) | Open access, CC BY-NC-ND | No (site blocks scripted download) |
| 8 | *Off-Road Navigation via Implicit Neural Representation of Terrain Traversability* (TRAIL) | arXiv:2511.18183; published in IEEE RA-L, 2026 (doi:10.1109/LRA.2026.3685928) | Preprint on arXiv; published version paywalled | **Yes**, the arXiv preprint |

## Where to get the missing ones

Open in a normal browser and save into this folder using the same numbering
(for example `02_terrain_classification_offroad_survey_2025.pdf`):

- 1: https://doi.org/10.1002/rob.22461 (needs institutional access)
- 2: https://onlinelibrary.wiley.com/doi/pdfdirect/10.1002/rob.22586
- 4: https://doi.org/10.1016/j.dt.2026.04.016
- 5: https://www.mdpi.com/2075-1702/10/5/375/pdf
- 7: https://doi.org/10.1016/j.geits.2026.100444

## Notes on what is stored

- **3** — the implementation can be inspected as well:
  https://github.com/leggedrobotics/wild_visual_navigation
- **6** — a documentation page, not a paper. The snapshot will go stale as Nav2
  changes; the controller parameters that matter are in
  `drishti-ugv/ugv_ws/src/drishti_bringup/config/nav2.yaml`.
- **8** — the arXiv preprint can differ from the version published in RA-L. Cite
  the published version; read the preprint for the content.

## Why the PDFs are not in git

Republishing a paper is a licence question, and a repository tends to become
public eventually. The arXiv preprint (8) is under arXiv's non-exclusive
distribution licence, which lets arXiv distribute it but does not let anyone else
republish it; the Nav2 snapshot (6) should be checked against the documentation's
terms; the ND and NC-ND papers (2, 4, 7) allow redistribution only unmodified and,
for NC, non-commercially. Keeping only this list avoids the question.
