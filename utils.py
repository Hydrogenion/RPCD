from termcolor import colored
def grey_print(x):
    if x[-1]==':':
        print(colored(x, "grey"),end=' ')
    else:
        print(colored(x, "grey"))

def red_print(x):
    print(colored(x, "red"))


def green_print(x):
    if x[-1]==':':
        print(colored(x, "green"),end=' ')
    else:
        print(colored(x, "green"))


def yellow_print(x):
    if x[-1]==':':
        print(colored(x, "yellow"),end=' ')
    else:
        print(colored(x, "yellow"))


def blue_print(x):
    print(colored(x, "blue"))


def magenta_print(x):
    print(colored(x, "magenta"))


def cyan_print(x):
    print(colored(x, "cyan"))


def white_print(x):
    print(colored(x, "white"))

