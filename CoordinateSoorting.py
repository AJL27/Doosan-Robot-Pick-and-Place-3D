import numpy as np
import parser

# Adds a Layer Number for each row to sort individual layers
def layeringRows(arr, column):
    layeredArr = []
    prevRow = arr[0]
    layer = 0
    rows = 0
    layers = [0]
    for row in arr:
        if row[column] == prevRow[column]:
            row_with_layer = list(row) + [layer]
            rows += 1
        else:
            layer += 1
            layers.append(rows)
            rows += 1
            row_with_layer = list(row) + [layer]
        layeredArr.append(row_with_layer)
        prevRow = row
    layers.append(rows)
    return layeredArr, layers


# Creates a partition for quicksort
def partition(array, low, high):
    pivot = array[high]
    i = low - 1
    for j in range(low, high):
        if array[j] <= pivot:
            i = i + 1
            (array[i], array[j]) = (array[j], array[i])
    (array[i + 1], array[high]) = (array[high], array[i + 1])
    return i + 1


# Quicksort Algorithm
def quickSort(array, low, high):
    if low < high:
        pi = partition(array, low, high)
        quickSort(array, low, pi - 1)
        quickSort(array, pi + 1, high)


# Final Sort Algorithm to sort each axis
def sortEachLayer(arr, ranges):
    for i in range(len(ranges) - 1):
        quickSort(arr, ranges[i], ranges[i + 1] - 1)
    return arr


# Calling the functions together in order to sort the coordinates
def SortingCoordinates(file):
    voxelData = parser.read_vox_file(file)
    voxelSize = voxelData['size']
    voxels = voxelData['voxels']
    coords = np.delete(voxels, 3, 1)

    semiSortedCoordinates, xLayers = layeringRows(coords, 2)  # Layers each row based on Z Coordinate
    sortEachLayer(semiSortedCoordinates, xLayers)  # Sorts X Coordinates
    semiSortedCoordinates, yLayers = layeringRows(semiSortedCoordinates, 0)  # Layers each row based on X coordinate
    sortedCoordinates = sortEachLayer(semiSortedCoordinates, yLayers)  # Sorts Y Coordinates
    finalCoordinates = np.delete(sortedCoordinates, 4, 1)
    finalCoordinates = np.delete(finalCoordinates, 3, 1)

    return finalCoordinates