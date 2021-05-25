Title: Dreaming of Macintosh
Link: https://www.youtube.com/watch?v=ydvIvywyh7Y
Effects: 
- Depth of field (all frames)
- Image texture mapping: on floor, corner cylinder, and rectangles in prism
- Glossy reflection: on floor and corner cylinder
- Oren-nayar: helios bust and greek columns 
- Cook-torrance: trapdoor panels 
- Motion blur: during falling sequence
- Perlin noise: cloud textures 
- Area lights: rectangle panels on ceiling

Instructions:
- Dropbox folder with necessary elements here: https://www.dropbox.com/sh/5dn9konjhygv55i/AABJpQpPL7oGBaQXoW5TDgl8a?dl=0
- Add the folders "ads" and "models" to the same directory as the rest of the files
- run make
- run "./render" for representative frame 
- run "./render frame n" to generate frame n with low res and distributed effects off

NOTE: Project is built using c++17 and won't compile for any older versions