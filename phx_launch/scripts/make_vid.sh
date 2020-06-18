mkdir -p tmp
mv ~/.ros/frame*.jpg tmp
ffmpeg -framerate 7 -i tmp/frame%04d.jpg -c:v libx264 -profile:v high -crf 20 outs/output.mp4
rm -rf tmp
rm outs/debug.bag
