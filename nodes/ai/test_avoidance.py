import matplotlib.pyplot as plt
from soccerobjects import Point2D, Vector2D, Avoidance

# obstacles = [
	# Point2D(-2, -0.1),
	# Point2D(-1, 0.18),
	# Point2D(0, 0),
	# Point2D(1, -0.3)
# 			]
# obstacles = [
	# Point2D(-2, -0.1),
	# Point2D(-1, 0.12),
	# Point2D(0, 0.1),
	# Point2D(1, -0.2)
# 			]
obstacles = [
	Point2D(-0.4, -0.4),
	Point2D(-0.3, -0.3),
	Point2D(-0.2, -0.2),
	Point2D(-0.1, -0.1),
	Point2D( 0.0,  0.0),
	Point2D( 0.1,  0.1),
	Point2D( 0.2,  0.2),
	Point2D( 0.3,  0.3),
	Point2D( 0.4,  0.4)
]
# obstacles = [Point2D(-2, 0.1), Point2D(-2.01, -0.25)]
a = Avoidance(*obstacles)
current = Point2D(-2.9, 0)
end = Point2D(2.9, 0)
x = [current.x]
y = [current.y]
move_dist = 0.1

while current.x != end.x or current.y != end.y:
	avoid_end = a.avoid(current, end, False)
	v = Vector2D.from_points(current, avoid_end)
	if v.get_length() > move_dist:
		v.set_length(move_dist)
	current.x += v.x
	current.y += v.y
	x.append(current.x)
	y.append(current.y)

plt.plot(x, y, 'ro')
plt.plot([o.x for o in obstacles],
	[o.y for o in obstacles], 'bo')
plt.axis([-3, 3, -3, 3])
plt.show()
