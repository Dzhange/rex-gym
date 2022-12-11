import numpy as np
import pybullet as p
import math
import numpy as np
import pybullet_data


def get_point_cloud(depth, width, height, view_matrix, proj_matrix):
    # based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer

    # get a depth image
    # "infinite" depths will have a value close to 1    

    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
    y *= -1.
    x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
    h = np.ones_like(z)

    pixels = np.stack([x, y, z, h], axis=1)
    # filter out "infinite" depths
    pixels = pixels[z < 0.99]
    pixels[:, 2] = 2 * pixels[:, 2] - 1

    # turn pixels to world coordinates
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3: 4]
    points = points[:, :3]

    return points


def write_pointcloud(points, file_name):

    N = points.shape[0]
    f = open(file_name, "w")
    for i in range(N):
        f.write("{} {} {}\n".format(points[i, 0], points[i, 1], points[i, 2]))
    f.close()



def read_pointcloud(file_name):

   return np.loadtxt(file_name)


def getRayFromTo(mouseX, mouseY):
  width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
  )
  camPos = [
      camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
      camTarget[2] - dist * camForward[2]
  ]
  farPlane = 10000
  rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
  lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
                     rayForward[2] * rayForward[2])
  invLen = farPlane * 1. / lenFwd
  rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
  rayFrom = camPos
  oneOverWidth = float(1) / float(width)
  oneOverHeight = float(1) / float(height)

  dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
  dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
  rayToCenter = [
      rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
  ]
  ortho = [
      -0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
      -0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
      -0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
  ]

  rayTo = [
      rayFrom[0] + rayForward[0] + ortho[0], rayFrom[1] + rayForward[1] + ortho[1],
      rayFrom[2] + rayForward[2] + ortho[2]
  ]
  lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
  alpha = math.atan(lenOrtho / farPlane)
  return rayFrom, rayTo, alpha





def visualize_depth(depthBuffer, imgW, imgH):
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1, 1, 1, 1], radius=0.03)
    collisionShapeId = -1  #p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="duck_vhacd.obj", collisionFramePosition=shift,meshScale=meshScale)

    count = 0
    stepX = 1
    stepY = 1
    for w in range(0, imgW, stepX):
        for h in range(0, imgH, stepY):
            count += 1
            if ((count % 100) == 0):
                print(count, "out of ", imgW * imgH / (stepX * stepY))
            rayFrom, rayTo, alpha = getRayFromTo(w * (width / imgW), h * (height / imgH))
            rf = np.array(rayFrom)
            rt = np.array(rayTo)
            vec = rt - rf
            l = np.sqrt(np.dot(vec, vec))
            depthImg = float(depthBuffer[h, w])
            far = 1000.
            near = 0.01
            depth = far * near / (far - (far - near) * depthImg)
            depth /= math.cos(alpha)
            newTo = (depth / l) * vec + rf
            p.addUserDebugLine(rayFrom, newTo, [1, 0, 0])
            mb = p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=collisionShapeId,
                                baseVisualShapeIndex=visualShapeId,
                                basePosition=newTo,
                                useMaximalCoordinates=True)
            color = rgbBuffer[h, w]
            color = [color[0] / 255., color[1] / 255., color[2] / 255., 1]
            p.changeVisualShape(mb, -1, rgbaColor=color)
        p.addUserDebugLine(corners3D[0], corners3D[1], [1, 0, 0])
        p.addUserDebugLine(corners3D[1], corners3D[2], [1, 0, 0])
        p.addUserDebugLine(corners3D[2], corners3D[3], [1, 0, 0])
        p.addUserDebugLine(corners3D[3], corners3D[0], [1, 0, 0])
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)