const fs = require('fs');
const path = require('path');

const DIRECTORY = 'src/main/deploy/pathplanner/paths';

function flipY(point) {
    point.y = 8 - point.y;
}

for (const file of fs.readdirSync(DIRECTORY)) {
    if (file.endsWith('.path')) {
        const inputPath = path.join(DIRECTORY, file);

        // Clone JSON
        let json = JSON.parse(fs.readFileSync(inputPath));
        let clone = JSON.parse(JSON.stringify(json));

        for (let wp of clone.waypoints) {

            if (wp?.linkedName) delete(wp.linkedName);

            flipY(wp.anchor);

            if (wp?.prevControl) flipY(wp.prevControl);
            if (wp?.nextControl) flipY(wp.nextControl);
        }

        for (let rt of clone.rotationTargets) {
            if (rt?.rotationDegrees) rt.rotationDegrees *= -1;
        }

        if (clone.idealStartingState?.rotation) 
                clone.idealStartingState.rotation *= -1;

        if (clone.goalEndState?.rotation) 
                clone.goalEndState.rotation *= -1;

        // Build output filename
        const parsed = path.parse(inputPath);
        const outputPath = path.join(parsed.dir, parsed.name + "_mirrored" + parsed.ext);

        // Save mirrored version
        fs.writeFileSync(outputPath, JSON.stringify(clone, null, 2));

        console.log("Created:", outputPath);
    }
}