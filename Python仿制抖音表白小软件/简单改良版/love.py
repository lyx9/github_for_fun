# Python制作仿抖音表白神器
# 作者: Charles
# 公众号: Charles的皮卡丘
# 代码仅供学习交流，不得用于其他
import os
import sys
import random
import pygame
from pygame.locals import *


WIDTH, HEIGHT = 640, 480
BACKGROUND = (255, 255, 255)
if getattr(sys, 'frozen', False):
	CurrentPath = sys._MEIPASS
else:
	CurrentPath = os.path.dirname(__file__)
FONTPATH = os.path.join(CurrentPath, 'simkai.ttf')
MUSICPATH = os.path.join(CurrentPath, '1.mp3')
IMGPATH = os.path.join(CurrentPath, '1.png')


# 按钮
def button(text, x, y, w, h, color, screen):
	pygame.draw.rect(screen, color, (x, y, w, h))
	font = pygame.font.Font(FONTPATH, 20)
	textRender = font.render(text, True, (0, 0, 0))
	textRect = textRender.get_rect()
	textRect.center = ((x+w/2), (y+h/2))	
	screen.blit(textRender, textRect)


# 标题
def title(text, screen, scale, color=(255, 0, 0)):
	font = pygame.font.Font(FONTPATH, WIDTH//(len(text)*2))
	textRender = font.render(text, True, color)
	textRect = textRender.get_rect()
	textRect.midtop = (WIDTH/scale[0], HEIGHT/scale[1])
	screen.blit(textRender, textRect)


# 生成随机的位置坐标
def get_random_pos():
	x, y = random.randint(20, 620), random.randint(20, 460)
	return x, y


# 点击喜欢按钮后显示的页面
def show_like_interface(text, screen, color=(255, 0, 0)):
	screen.fill(BACKGROUND)
	font = pygame.font.Font(FONTPATH, WIDTH//(len(text)))
	textRender = font.render(text, True, color)
	textRect = textRender.get_rect()
	textRect.midtop = (WIDTH/2, HEIGHT/2)
	screen.blit(textRender, textRect)
	pygame.display.update()
	while True:
		for event in pygame.event.get():
			if event.type == QUIT:
				pygame.quit()
				sys.exit()


# 主函数
def main():
	pygame.init()
	screen = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
	pygame.display.set_caption('FROM一个喜欢你很久的小哥哥')
	clock = pygame.time.Clock()
	pygame.mixer.music.load(MUSICPATH)
	pygame.mixer.music.play(-1, 30.0)
	pygame.mixer.music.set_volume(0.25)
	unlike_pos_x = 330
	unlike_pos_y = 250
	unlike_pos_width = 100
	unlike_pos_height = 50
	unlike_color = (0, 191, 255)
	like_pos_x = 180
	like_pos_y = 250
	like_pos_width = 100
	like_pos_height = 50
	like_color = (0, 191, 255)
	running = True
	while running:
		screen.fill(BACKGROUND)
		img = pygame.image.load(IMGPATH)
		imgRect = img.get_rect()
		imgRect.midtop = int(WIDTH/1.3), HEIGHT//7
		screen.blit(img, imgRect)
		for event in pygame.event.get():
			if event.type == pygame.MOUSEBUTTONDOWN:
				mouse_pos = pygame.mouse.get_pos()
				if mouse_pos[0] < like_pos_x+like_pos_width+5 and mouse_pos[0] > like_pos_x-5 and\
					mouse_pos[1] < like_pos_y+like_pos_height+5 and mouse_pos[1] > like_pos_y-5:
					like_color = BACKGROUND
					running = False
		mouse_pos = pygame.mouse.get_pos()
		if mouse_pos[0] < unlike_pos_x+unlike_pos_width+5 and mouse_pos[0] > unlike_pos_x-5 and\
			mouse_pos[1] < unlike_pos_y+unlike_pos_height+5 and mouse_pos[1] > unlike_pos_y-5:
			while True:
				unlike_pos_x, unlike_pos_y = get_random_pos()
				if mouse_pos[0] < unlike_pos_x+unlike_pos_width+5 and mouse_pos[0] > unlike_pos_x-5 and\
					mouse_pos[1] < unlike_pos_y+unlike_pos_height+5 and mouse_pos[1] > unlike_pos_y-5:
					continue
				break
		title('小姐姐，我观察你很久了', screen, scale=[3, 8])
		title('做我女朋友好不好呀', screen, scale=[3, 4])
		button('好呀', like_pos_x, like_pos_y, like_pos_width, like_pos_height, like_color, screen)
		button('算了吧', unlike_pos_x, unlike_pos_y, unlike_pos_width, unlike_pos_height, unlike_color, screen)
		pygame.display.flip()
		pygame.display.update()
		clock.tick(60)
	show_like_interface('我就知道小姐姐你也喜欢我~', screen, color=(255, 0, 0))


if __name__ == '__main__':
	main()