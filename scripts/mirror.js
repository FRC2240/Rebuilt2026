const fs = require('fs');
const path = require('path');

const DIRECTORY = 'src/main/deploy/pathplanner/paths';

function flipY(point) {
    point.y = 8.07 - point.y;
}

const files = fs.readdirSync(DIRECTORY);
for (const file of files) {
    if (file.endsWith('.path') && (file.includes("Right") || file.includes("Left"))) {
        const inputPath = path.join(DIRECTORY, file);

        // Build output filename
        const parsed = path.parse(inputPath);
        let newName = parsed.name + parsed.ext;
        if (newName.includes("Left")) {
            newName = newName.replace("Left", "Right")
        } else if (newName.includes("Right")) {
            newName = newName.replace("Right", "Left")
        }
        
        // Skip paths that exist
        if (files.includes(newName)) {
            console.log(`WARNING: ${newName} already exists. Skipping...`)
            continue;
        }

        const outputPath = path.join(parsed.dir, newName);

        // Clone JSON
        let json = JSON.parse(fs.readFileSync(inputPath));
        let clone = JSON.parse(JSON.stringify(json));

        for (let wp of clone.waypoints) {
            if (wp?.linkedName) {
                if (wp.linkedName.includes("right")) {
                    wp.linkedName = wp.linkedName.replace("right", "left")
                }else if (wp.linkedName.includes("left")) {
                    wp.linkedName = wp.linkedName.replace("left", "right")
                }
            }

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


        // Save mirrored version
        fs.writeFileSync(outputPath, JSON.stringify(clone, null, 2));

        console.log("Created:", outputPath);
    }
}