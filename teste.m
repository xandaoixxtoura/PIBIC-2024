
close all
clear all
clc

load('test2.mat')
img_flipped_vert = flipud(maps);

imshow(uint8(img_flipped_vert))
hold on
[M,N]=size(experimento.mapa_descoberto);
% plot(experimento.fronteirax,M-experimento.fronteiray,'.r','MarkerSize',5);
