import Interpolation as interp
import CoordinateSoorting as cs
import robotCommands as rc

INTOMM = 25.4
VOXSIZE = 20.23

PTVEC = (
    [0, 0, 0], [0, 0, 101], [0, 0, 202],
    [0, 101, 0], [0, 101, 101], [0, 101, 202],
    [0, 202, 0], [0, 202, 101], [0, 202, 202],
    [101, 0, 0], [101, 0, 101], [101, 0, 202],
    [101, 101, 0], [101, 101, 101], [101, 101, 202],
    [101, 202, 0], [101, 202, 101], [101, 202, 202],
    [202, 0, 0], [202, 0, 101], [202, 0, 202],
    [202, 101, 0], [202, 101, 101], [202, 101, 202],
    [202, 202, 0], [202, 202, 101], [202, 202, 202]
)


XYVEC = (
    [0, 0, 0], [404, 0, 0], [404, 404, 0], [0, 404, 0],
    [0, 0, 202], [404, 0, 202], [404, 404, 202], [0, 404, 202],
    [0, 0, 101], [404, 0, 101], [404, 404, 101], [0, 404, 101],
    [202, 0, 0], [404, 202, 0], [202, 404, 0], [0, 202, 0], [202, 202, 0],
    [202, 0, 202], [404, 202, 202], [202, 404, 202], [0, 202, 202], [202, 202, 202],
    [202, 0, 101], [404, 202, 101], [202, 404, 101], [0, 202, 101], [202, 202, 101]
)

JTVEC = (
    [80.15, -58.32, -66.86, 0, -55.42, -9.12],
    [100.55, -58.32, -66.86, 0, -56.01, 11],
    [106.62, -37.67, -114.29, 0, -29.67, 16.68],
    [74.9, -37.74, -114.29, 0, -28.67, -15.15],
    [80.15, -47.66, -66.86, 0, -60.98, -9.12],
    [100.04, -47.66, -66.86, 0, -63.97, 11],
    [106.62, -21.71, -114.29, 0, -40.45, 16.68],
    [74.9, -22.32, -114.29, 0, -41.45, -15.15],
    [80.15, -52.99, -66.86, 0, -58.2, -9.12],
    [100.04, -52.99, -66.86, 0, -59.99, 11],
    [106.62, -29.69, -114.29, 0, -35.06, 16.68],
    [74.9, -30.03, -114.29, 0, -35.06, -15.15],
    [90.34, -57.5, -68.97, 0, -55.44, 1.64],
    [102.67, -46.44, -93.65, 0, -40.8, 11.75],
    [90.17, -37.03, -116.79, 0, -27.87, 1.17],
    [78.01, -46.78, -93.13, 0, -40.8, -10],
    [90.35, -45.44, -96.13, 0, -38.84, 1.87],
    [90.34, -47.04, -68.97, 0, -61.04, 1.64],
    [102.67, -33.86, -93.65, 0, -48.46, 11.75],
    [90.17, -20.71, -116.79, 0, -38.75, 1.17],
    [78.01, -34.48, -93.13, 0, -48.32, -10],
    [90.35, -33.04, -96.13, 0, -47.64, 1.87],
    [90.34, -52.27, -68.97, 0, -58.24, 1.64],
    [102.67, -40.15, -93.65, 0, -44.63, 11.75],
    [90.17, -28.87, -116.79, 0, -33.31, 1.17],
    [78.01, -40.63, -93.13, 0, -44.56, -10],
    [90.35, -39.24, -96.13, 0, -43.24, 1.87]
)

#Where the robot picks up each block
bcvec = []  # Pickup Block Coordinates
startx = INTOMM / 2
starty = 404 - INTOMM / 2
startz = 3
bcvec.append([startx, starty, startz])

for i in range(1, numberOfBlocks, 1):
    if i <= 18:
        starty -= INTOMM * 1.35
        if i % 6 == 0:
            startx += INTOMM * 3
            starty = 404 - INTOMM / 2
    else:
        starty -= INTOMM * 1.35
        if i % 12 == 6:
            startx += INTOMM * 3
            starty = 404 - INTOMM / 2
    bcvec.append([startx, starty, startz])

finalbcvec = []

for point in bcvec:
    x, j = interp.map(point[0], point[1], point[3], PTVEC, XYVEC, JTVEC)
    finalbcvec.append([round(num, 2) for num in j])

#Where the robot places the blocks
finaljtvec = []  # Final Joint Angles
finalxyvec = []  # Final XY Coordinates
coordinates = cs.SortingCoordinates(input("Enter a file name"))
numberOfBlocks = len(coordinates)

voxelCoordinates = []
for point in coordinates:
    if point[2] == 0:
        transformed_point = (
            float(point[0]) * VOXSIZE + VOXSIZE / 2.5,
            float(point[1]) * VOXSIZE + VOXSIZE / 2.5,
            3
        )
    else:
        transformed_point = (
            float(point[0]) * VOXSIZE + VOXSIZE / 2.5,
            float(point[1]) * VOXSIZE + VOXSIZE / 2.5,
            float(point[2]) * VOXSIZE + VOXSIZE / 2.5
        )
    voxelCoordinates.append(transformed_point)


for point in voxelCoordinates:
    x, j = map(point[0], point[1], point[3], PTVEC, XYVEC, JTVEC)
    finalxyvec.append([round(num, 2) for num in x])
    finaljtvec.append([round(num, 2) for num in j])


#Getting two of the side blocks, the one above it and to the right as those are the ones already placed
sides = []
for i in range(numberOfBlocks):
    p1 = [finalxyvec[i][0], round(finalxyvec[i][1] - 25.4, 2), finalxyvec[i][3]]
    p2 = [round(finalxyvec[i][0] - 25.4, 2), finalxyvec[i][1], finalxyvec[i][3]]
    sides.append([p1, p2])


#Sending Messages to the robot
rc.sendmessage(rc.openGripper)
rc.received_message()

for i in range(numberOfBlocks):

    moveToCube = [b'moveto', finalbcvec[i]]
    moveToPlace = [b'moveto', finaljtvec[i]]

    rc.sendmessage(moveToCube)
    rc.received_message()
    rc.sendmessage(rc.movedown)
    rc.received_message()
    rc.sendmessage(rc.closeGripper)
    rc.received_message()
    rc.sendmessage(rc.moveup)
    rc.received_message()
    rc.sendmessage(moveToPlace)
    rc.received_message()
    print("Coords: ", finalxyvec[i])
    print("Sides: ", sides[i])
    if sides[i][0] in finalxyvec[:i] and sides[i][1] in finalxyvec[:i]:
        rc.sendmessage(rc.openGripper)
    elif sides[i][1] in finalxyvec[:i]:
        rc.sendmessage(rc.rotategripper)
        rc.received_message()
        rc.sendmessage(rc.movedown)
        rc.received_message()
        rc.sendmessage(rc.openGripper)
        rc.received_message()
        rc.sendmessage(rc.rotategripper)
        rc.received_message()
    else:
        rc.sendmessage(rc.movedown)
        rc.received_message()
        rc.sendmessage(rc.openGripper)
        rc.received_message()

