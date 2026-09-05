// tsci renders on its default warm-grey canvas (rgb(245, 241, 237)), set in two
// places: the root <svg style> and the .boundary class. Both have to change or
// the paper stays grey behind a white frame. Run after `tsci export
// -f schematic-svg`; see the "svg" script in package.json.
import { readFileSync, writeFileSync } from "node:fs";

// One level up: the committed exports live in docs/schemes/wiring/, beside
// this toolchain folder rather than inside it.
const FILE = "../harness.schematic.svg";
const CANVAS = /rgb\(245, ?241, ?237\)/g;

const svg = readFileSync(FILE, "utf8");
const hits = svg.match(CANVAS)?.length ?? 0;
if (hits === 0) throw new Error(`no canvas colour found in ${FILE} -- did tsci change its default?`);

writeFileSync(FILE, svg.replace(CANVAS, "rgb(255, 255, 255)"));
console.log(`whitened ${hits} canvas colour reference(s) in ${FILE}`);
