# Software License Agreement (BSD License) 
#
# Copyright (c) 2011, Yaskawa America, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 	* Redistributions of source code must retain the above copyright
# 	notice, this list of conditions and the following disclaimer.
# 	* Redistributions in binary form must reproduce the above copyright
# 	notice, this list of conditions and the following disclaimer in the
# 	documentation and/or other materials provided with the distribution.
# 	* Neither the name of the Yaskawa America, Inc., nor the names 
#	of its contributors may be used to endorse or promote products derived
#	from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

firstPoint = []
firstPoint.append(int(raw_input("Enter starting (S) value ==> ")))
firstPoint.append(int(raw_input("Enter starting (L) value ==> ")))
firstPoint.append(int(raw_input("Enter starting (U) value ==> ")))
firstPoint.append(int(raw_input("Enter starting (R) value ==> ")))
firstPoint.append(int(raw_input("Enter starting (B) value ==> ")))
firstPoint.append(int(raw_input("Enter starting (T) value ==> ")))
firstPoint.append(int(raw_input("Enter starting (E) value ==> ")))
firstPoint.append(int(raw_input("Enter starting (8th axis) value ==> ")))

inc = []
inc.append(int(raw_input("Enter (S) increment ==> ")))
inc.append(int(raw_input("Enter (L) increment ==> ")))
inc.append(int(raw_input("Enter (U) increment ==> ")))
inc.append(int(raw_input("Enter (R) increment ==> ")))
inc.append(int(raw_input("Enter (B) increment ==> ")))
inc.append(int(raw_input("Enter (T) increment ==> ")))
inc.append(int(raw_input("Enter (E) increment ==> ")))
inc.append(int(raw_input("Enter (8th axis) increment ==> ")))

numPoints = int(raw_input("Enter number of points ==> "))

filename = "trajectory.txt"
f = open(filename, 'w')

for i in range(numPoints):
    line = ""
    line += "{"
    if i == 0:
        line += "{"
    for j in range(7):
        line += str(firstPoint[j]+inc[j]#i)
        line += ", "
    line += str(firstPoint[7]+inc[7]#i)
    if i == (numPoints-1):
        line += "}};"
    else:
        line += "},"
    # print line
    f.write(line)
    if i != (numPoints-1):
        f.write("\n")
f.close()
print "Trajectory written to",filename




