import math
#Prob1
def distanceBetweenPoints(point1,point2):
  x1,y1 = point1
  x2,y2 = point2
  dx,dy = x2-x1,y2-y1
  distance = math.sqrt(dx**2 + dy**2)
  return distance

def check(x,y,cx,cy,r):
  point1 = (x,y)
  point2 = (cx,cy)
  result = distanceBetweenPoints(point1,point2)
  return result - r
threats = [(6,5,3),
(10, 15,2),
(14, 11,1),
(22, 5,4),
(22, 13,2),
(29, 11,2),
(28, 17,3),
(32, 17,1),
(35, 5,3),
(34, 10,4)]
#might need to change the format of threats list
guess = [(3,12),(11, 18), (17, 18), (23, 18), (29, 18), (35, 18),(40,13)]
distanceList = []
for i in range(len(guess)-1):
  point1 = guess[i]
  point2 = guess[i+1]
  distance = distanceBetweenPoints(point1,point2)
  distanceList.append(distance)
print(distanceList)

#Objective Function
#PART 1 OF OBJECTIVE FUNCTION
A = 0
for i in range(len(guess)-1):
  point1 = guess[i]
  point2 = guess[i+1]
  print("----------------------------------------------")
  print("for points: " + str(point1) + str(point2))
  lj = distanceBetweenPoints(point1, point2)
  oMax=1
  kMax = 1+(-(-lj//oMax))
  lambdaThing = 1/kMax
  #This is T(uk)
  k=0
  lji = 0
  kMax = (lambdaThing)**-1
  uKXPrev = 0
  uKYPrev = 0
  for j in range(len(threats)):
    cPoints = (threats[j])[0:2]
    radius = (threats[j])[-1]
    print("Threat " + str(j+1))
    print("Center Points: " + str(cPoints))
    print("Radius: " + str(radius))

    for a in range(int(kMax)):
      uKX = point1[0] + a * lambdaThing * (point2[0]-point1[0]) # could use slice to get the numbers
      uKY = point1[1] + a * lambdaThing * (point2[1]-point1[1])
      resultCur = check(uKX,uKY,cPoints[0],cPoints[1],radius)
      resultPrev = check(uKXPrev,uKYPrev,cPoints[0],cPoints[1],radius)
      print('X:' + str(uKX) + ' Y:' + str(uKY) )
      print(resultCur)
      if(a == 0 and resultCur <= 0):
        lambdaB = 0
      if(a > 0 and resultCur <=0 and resultPrev > 0):
        weirdK = resultCur/(resultCur-resultPrev)
        lambdaB = (k-weirdK)*lambdaThing
      if (a > 0 and resultCur > 0 and resultPrev <= 0):
        weirdK = resultCur/(resultCur-resultPrev)
        lambdaE = (k - weirdK)*lambdaThing
        lji = lji + (lambdaE - lambdaB)*lj
      if (a == kMax and resultCur <= 0):
        lji = lji + (1- lambdaB)*lj
      uKXPrev = uKX
      uKYPrev = uKY
    print("lji: " + str(lji))
  A = A + lj + lji
print(A)

#Part 2 of the Objective Function
# gonna assume lengthj^T is just lengthj because I have no idea what T is
#angleJ = math.acos((lengthj^T * lengthj+1)/(math.abs(lengthj)* math.abs(lengthj+1)))
B = 0
angleMax = 31
penaltyTermV = 0.0001
for i in range(len(guess)-1):
  point1 = guess[i]
  point2 = guess[i+1]
  print("----------------------------------------------")
  print("for points: " + str(point1) + str(point2))
  lj = distanceBetweenPoints(point1, point2)
  j = 1
  for j in range(len(guess) - 1):
    newPoint1 = guess[j]
    newPoint2 = guess[j+1]
    ljNew = distanceBetweenPoints(newPoint1,newPoint2)
    angleJ = math.acos((lj * lj+1)/(abs(lj)* abs(lj+1)))
    result = (angleMax - angleJ)
    if result > 0:
      result = 0
    B = B + penaltyTermV*(result**2)
  print(B)

#Part 3 of the Objective Function
lmin = 1
penaltyTermU = 0.01
C = 0
for i in range(len(guess)-1):
  point1 = guess[i]
  point2 = guess[i+1]
  print("----------------------------------------------")
  print("for points: " + str(point1) + str(point2))
  lj = distanceBetweenPoints(point1, point2)
  result = (lj - lmin)
  if result > 0:
    result = 0
  C = C + penaltyTermU*(result**2)
  print(C)

  #COST
Cost = A + B + C
print(Cost)


