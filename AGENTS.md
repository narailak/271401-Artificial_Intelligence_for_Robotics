# Workspace instructions

## Always use Ponytail

- Apply the `ponytail` skill to every request in this workspace and its subdirectories, including coding, debugging, reviews, planning, and explanations.
- At the start of each session, read `/home/aorus-ubun/.codex/skills/ponytail/SKILL.md`. Keep its instructions active on every subsequent turn; do not reread it unnecessarily.
- Default to `full` mode unless the user explicitly selects another level or turns Ponytail off. Follow the user's latest instruction.
- If the skill is missing or unreadable, report that rather than claiming it is active.
- Ponytail never overrides higher-priority instructions, the user's requirements, or checks needed for safe robot operation.

Installed source: [DietrichGebert/ponytail](https://github.com/DietrichGebert/ponytail/tree/main/skills/ponytail).

## Always use Caveman

- Apply the `caveman` skill to every request in this workspace and its subdirectories, alongside Ponytail.
- At the start of each session, read `/home/aorus-ubun/.codex/skills/caveman/SKILL.md`. Keep its instructions active on subsequent turns without unnecessary rereading.
- Keep replies concise and use the user's language. Preserve technical details, safety checks, and clear instructions.
- Follow the user's latest instruction if they change or disable the mode.
- If the skill is missing or unreadable, report it rather than claiming it is active.
