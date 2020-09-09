#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Requirements:
########################################################################################################################

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
