v = VideoReader('E:/data/jqw/XiaoYing_Video_1665459723143.mp4');

for i=1:150,
    frame = read(v, i);
    file_path = sprintf('e:/data/jqw/video_frames/%d.png', i);
    imwrite(frame, file_path);
end