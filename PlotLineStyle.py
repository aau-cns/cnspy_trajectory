class PlotLineStyle():
    linecolor = 'r'  # b,g,r,c,m,y,k,w
    linewidth = 1
    linestyle = '-'  # '-', '--', '-.', ':'
    marker = '.'  # '.', ',', 'o', 'v', '^', '<', '>', '1..4', 's', 'p'
    markersize = 12
    markerfacecolor = 'blue'
    label = 'x'

    def __init__(self, linecolor='r', linewidth=1, linestyle='-', marker='o', markersize=12,
                 markerfacecolor='blue', label='x'):
        self.linecolor = linecolor
        self.linewidth = linewidth
        self.linestyle = linestyle
        self.marker = marker
        self.markersize = markersize
        self.markerfacecolor = markerfacecolor
        self.label = label
