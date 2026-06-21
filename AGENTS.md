# AGENTS.md

This is a Nix-flake-powered autonomous RC car repo running on :

- Motor + ESC — Hobbywing QUICRUN 10BL120 (120A) + 540 V3 4.5T sensored
  brushless motor
- Steering — INJORA INJS022 22KG digital servo
- LiDAR — Slamtec RPLiDAR A1
- Ultrasonics — 3× HC-SR04 (left, front, right)
- IMU — (MPU6050-type from the link)
- Brain — Raspberry Pi 5 + active cooler + Geekworm X1200 UPS HAT (2× 18650)
- Body — Carbon-fiber PLA print, custom 3D-printed mounts

The ultrasonics are not in use for the moment.

This repository uses the Jujutsu version control system (see the `/jujutsu`
skill for details).

## How things work

OS related changes are made in `hosts/nfe/configuration.nix` were we declare
services used. The software running the car is declared as a systemd.service in
`modules/car-service.nix` and the real-time patch in `modules/preempt-rt.nix`
The said software is stored in `packages/nfe-car` and built in rust.

the nfe-car package includes the `car` runtime binary, `car-diag` diagnostics,
`car-tune` CMA-ES tuning, and `nfe-arm` StartGate arming helper.

Deployment of the car OS and binaries is made using deploy-rs see `flake.nix`

## Key conventions

- Smallest reasonable changes. Do not refactor unrelated code.
- Match the style of surrounding code even if it differs from standard guides.
- Do not remove code comments unless they are provably false.
- Do not add temporal references in comments (e.g. "recently added").
- No trailing whitespace, including on blank lines.
- Comments should describe "why", not "what".

## Version control

This repo uses [Jujutsu](https://jj-vcs.github.io/) (`jj`) instead of `git` for
version control. Commit frequently and make small, focused commits.

### Commit message format

Use [Conventional Commits](https://www.conventionalcommits.org/) formatting:

- Start the subject line with a type prefix: `docs:`, `fix:`, `chore:`, `test:`,
  `refactor:`, `feat:`, etc.
- Optionally scope the prefix (e.g., `refactor(recorder):`, `fix(pid):`); if the
  changes affect a single aspect, use the aspect name as the scope.
- The rest of the subject line should start with a verb in the imperative form;
  ie. "add", "teach", "fix" etc.
- Keep subject lines under 72 columns.
- In the commit body, hard-wrap to 80 columns.
- Use Markdown formatting for _bold_, _italics_, `code`, and fenced code blocks.
- Describe _what_ changed as concisely as possible; fit it in the subject if you
  can, but feel free to continue concisely in the body if fitting it all in the
  subject is not possible.
- Use the body to explain the motivation for the change was made and why the
  particular approach was chosen; you should include info on the alternatives
  considered, and why they were not chosen.

## Markdown

When writing Markdown, do not hard-wrap long lines.

H2 Update docs

Whenever you make a change make sure to update all relevant docs if needed
(`README.md`, etc...)
