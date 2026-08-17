# Collection Target Dropdowns Design

## Goal

Replace the free-form collection target inputs and startup target arguments with two runtime dropdowns. Both fixed stage-2 collection and full staged collection must use the selected left- and right-hand targets when writing prompts and episode metadata.

## Target Catalog

Both dropdowns use the same ordered catalog and add an empty option first:

1. Coca-Cola
2. Daily C Grape Juice
3. Guangming Probiotic Milk
4. Daily C Orange Juice
5. AD Calcium Milk
6. Robuk Velvet Latte
7. Aojiru
8. HK Orange Fanta
9. Taro Milk
10. Yili Peach Yogurt
11. NEVER Coconut Latte
12. Yili Strawberry Yogurt
13. Wanglaoji
14. Sprite
15. Yakult
16. Dahongpao Milk Tea

The initial selection for both hands is empty. The server rejects any non-empty target outside this catalog.

## Entry Points and Shared Components

- `scripts/collection/collect_mobile_pipeline_web_fixed_stage.sh` remains the fixed stage-2 wrapper, but its target arguments and help text are removed.
- `scripts/collection/collect_mobile_pipeline_web_staged.sh` no longer parses target arguments or derives startup arm mode from them. It starts both targets empty and creates a target-independent stage preset.
- `scripts/collection/collect_mobile_episode_web.sh` owns the shared dropdown UI, validates runtime selections, exposes them in status, refreshes stage prompts, and writes selected targets into episode metadata.
- A focused Python helper owns the exact catalog, validation, and prompt formatting so fixed-stage and full-stage metadata cannot diverge.
- `scripts/collection/collection_episode_metadata.py` delegates its fallback grasp instruction to the shared helper.

## Prompt Rules

Stage 2 uses the following exact rules:

| Left | Right | Prompt |
|---|---|---|
| empty | empty | empty string |
| `L` | empty | `Grasp L with the left hand.` |
| empty | `R` | `Grasp R with the right hand.` |
| `L` | `R` | `Grasp L with the left hand. and Grasp R with the right hand.` |

Stages 1, 3, and 5 retain their existing text. Stage 4 retains its existing `Place` behavior while using the runtime selection:

| Left | Right | Prompt |
|---|---|---|
| empty | empty | empty string |
| `L` | empty | `Place L into the cart with the left hand.` |
| empty | `R` | `Place R into the cart with the right hand.` |
| `L` | `R` | `Place L into the cart with the left hand, then place R into the cart with the right hand.` |

The full instruction remains the newline-joined five-stage instruction. An explicitly empty stage-2 or stage-4 description remains empty in saved metadata rather than being replaced with `Stage 2` or `Stage 4`.

## Runtime Data Flow

1. The collection status response includes the ordered target catalog and current left/right values.
2. The browser builds both dropdowns from that catalog and selects the current values. It submits them through the existing `/config` request before manual collection starts or topic checks.
3. The configuration handler trims and validates both values. It refuses configuration changes while recording or awaiting quality review, preserving the existing lock behavior.
4. Applying a valid selection updates the target status fields and refreshes stage labels before the next episode starts.
5. Automatic state-machine collection reuses the already-applied target values when it switches staged mode; it does not overwrite them.
6. Metadata target arguments contain only non-empty selections in left-then-right order. Fixed stage-2 fallback metadata and full staged metadata use the same formatter.

Selections may change between episodes without restarting either entry point.

## UI Behavior

The labels become `左手目标物品` and `右手目标物品`. Both controls are `<select>` elements, show `空` first, and contain the 16 exact English values. They are disabled while recording or while a quality review is pending, matching the other collection configuration controls.

## Error Handling

- Startup target flags, positional target values, and target environment defaults no longer select prompt objects.
- Unknown extra positional arguments fail with the existing unrecognized-argument path.
- A forged `/config` request containing a target outside the catalog is rejected and does not change the current selection.
- An empty selection is valid for either or both hands.

## Tests

Automated tests cover the exact catalog and order, target validation, all four grasp combinations, all four place combinations, fixed-stage fallback metadata, full-stage prompt placeholders, dropdown markup/data flow, removal of terminal target parsing, and preservation of stages 1, 3, and 5. Shell syntax and Python compilation checks cover all modified executable files.
