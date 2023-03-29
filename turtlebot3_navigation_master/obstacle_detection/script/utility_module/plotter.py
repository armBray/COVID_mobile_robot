import numpy as np
import matplotlib.pyplot as plot

color_array = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
def create_polar_plot(dictionary):
    plot.axes(projection='polar')
    key_list = dictionary.keys()
    item_list = dictionary.values()
    for i in range(len(key_list)):
        plot.polar(np.radians(key_list[i]),item_list[i],'.')
    plot.show()

def create_cartesian_plot(dictionary):
    key_list = dictionary.keys()
    item_list = dictionary.values()
    for i in range(len(key_list)):
        plot.plot(key_list[i],item_list[i],'.')
    plot.show()

def create_cartesian_plot_not_show(dictionary, index_color):
    key_list = dictionary.keys()
    item_list = dictionary.values()
    for i in range(len(key_list)):
        plot.plot(key_list[i],item_list[i],'.', color=color_array[index_color])

def create_polar_plot_not_show(dictionary, index_color):
    plot.axes(projection='polar')
    key_list = dictionary.keys()
    item_list = dictionary.values()
    for i in range(len(key_list)):
        plot.plot(np.radians(key_list[i]),item_list[i],'.', color=color_array[index_color])

def create_polar_plot_colored(dictionary):
    index_color = 0
    max_index_color = len(color_array) - 1
    for key in dictionary.keys():
        create_polar_plot_not_show(dictionary.get(key), index_color)
        if index_color != max_index_color:
            index_color = index_color + 1
        else:
            index_color = 0

    plot.show()

def create_cartesian_plot_colored(dictionary):
    index_color = 0
    max_index_color = len(color_array) - 1
    for key in dictionary.keys():
        create_cartesian_plot_not_show(dictionary.get(key), index_color)
        if index_color != max_index_color:
            index_color = index_color + 1
        else:
            index_color = 0

    plot.show()
