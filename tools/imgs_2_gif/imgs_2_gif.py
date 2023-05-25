import imageio
import os
import moviepy.editor as mp



def create_gif(image_list):
    with imageio.get_writer('output.gif', mode="I") as writer:
        i = 0
        for image_ in image_list:
            if i % 3 == 0:
                image = imageio.imread(image_)
                writer.append_data(image)
            i = i + 1


def create_img_list(target_dir):
    os.chdir(target_dir)
    image_list = []

    for file in os.listdir(os.curdir):
        if file.endswith(".jpg") or file.endswith(".png"):
            image_list.append(file)

    image_list = sorted(image_list, key=lambda x: int(x[5:-4]))
    return image_list


# img_list = []
# for i in range(26, 593):
#     img_list.append('C:/Users/xiaoying/AppData/Local/Temp/ScreenToGif/Recording/2021-08-13 14-45-48/'+str(i)+'.png')
#
# create_gif(img_list)
clip = mp.VideoFileClip("output.gif")
clip.write_videofile("output.mp4")