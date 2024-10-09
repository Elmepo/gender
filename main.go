package main

import (
	"fmt"
	"log"
	"math"
	// "math/rand/v2"
	"os"
	"time"

	"gender/mesh"

	"github.com/gdamore/tcell/v2"
)

func multiplyMatrixVector(i mesh.Point, m [][]float64, logger *log.Logger) mesh.Point {
	var o mesh.Point
	logger.Printf("mutDEBUG1")
	o.X = i.X * m[0][0] + i.Y * m[1][0] + i.Z * m[2][0] + m[3][0]
	logger.Printf("mutDEBUG2")
	o.Y = i.X * m[0][1] + i.Y * m[1][1] + i.Z * m[2][1] + m[3][1]
	logger.Printf("mutDEBUG3")
	o.Z = i.X * m[0][2] + i.Y * m[1][2] + i.Z * m[2][2] + m[3][2]
	logger.Printf("mutDEBUG4")
	var w float64
	w = i.X * m[0][3] + i.Y * m[1][3] + i.Z * m[2][3] + m[3][3]
	logger.Printf("mutDEBUG5")

	if w != 0.0 {
		o.X = o.X / w
		o.Y = o.Y / w
		o.Z = o.Z / w
	}
	logger.Printf("Matrix: %+v", m)
	logger.Printf("o: %+v", o)
	return o
}

func drawLine(a mesh.Point, b mesh.Point, s tcell.Screen, logger *log.Logger) {
	logger.Printf("Trying to print a line")

	ax := int(math.Round(a.X))
	ay := int(math.Round(a.Y))
	bx := int(math.Round(b.X))
	by := int(math.Round(b.Y))

	dx := int(math.Abs(float64(bx - ax)))
	dy := int(math.Abs(float64(by - ay)))
	logger.Printf("dx: %d, dy: %d", dx, dy)
	sx := 1
	sy := 1

	if ax > bx {
		sx = -1
	}

	if ay > by {
		sy = -1
	}

	err := dx - dy


	for {
		logger.Printf("ax: %d, ay: %d, bx: %d, by: %d, err: %d, -dy: %d, dx: %d", ax, ay, bx, by, err, -dy, dx)
		// s.SetContent(ax, ay, tcell.RuneBlock, nil, tcell.StyleDefault.Background(tcell.ColorBlack).Foreground(tcell.ColorBlack))
		s.SetContent(ax, ay, tcell.RuneBlock, nil, tcell.StyleDefault)

		if ax == bx && ay == by {
			break
		}

		// if (ax == bx || ax > bx) || (ay == by || ay > by) {
		// 	break
		// }

		e2 := 2 * err

		if e2 > -dy {
			err -= dy
			ax += sx
		}

		if e2 <= dx {
			err += dx
			ay += sy
		}

	}
}

func getColor(lum float64) (tcell.Style, rune) {
	style := tcell.StyleDefault
	pixel_bw := int(13.0 * lum)
	switch pbw := pixel_bw; pbw {
	case 0:
		return style.Background(tcell.ColorBlack).Foreground(tcell.ColorBlack), tcell.RuneBlock

	case 1:
		return style.Background(tcell.ColorBlack).Foreground(tcell.ColorDarkGrey), 0x2591
	case 2:
		return style.Background(tcell.ColorBlack).Foreground(tcell.ColorDarkGrey), 0x2592
	case 3:
		return style.Background(tcell.ColorBlack).Foreground(tcell.ColorDarkGrey), 0x2593
	case 4:
		return style.Background(tcell.ColorBlack).Foreground(tcell.ColorDarkGrey), 0x2588

	case 5:
		return style.Background(tcell.ColorDarkGrey).Foreground(tcell.ColorGrey), 0x2591
	case 6:
		return style.Background(tcell.ColorDarkGrey).Foreground(tcell.ColorGrey), 0x2592
	case 7:
		return style.Background(tcell.ColorDarkGrey).Foreground(tcell.ColorGrey), 0x2593
	case 8:
		return style.Background(tcell.ColorDarkGrey).Foreground(tcell.ColorGrey), 0x2588

	case 9:
		return style.Background(tcell.ColorGrey).Foreground(tcell.ColorWhite), 0x2591
	case 10:
		return style.Background(tcell.ColorGrey).Foreground(tcell.ColorWhite), 0x2592
	case 11:
		return style.Background(tcell.ColorGrey).Foreground(tcell.ColorWhite), 0x2593
	case 12:
		return style.Background(tcell.ColorGrey).Foreground(tcell.ColorWhite), 0x2588
	default:
		return style.Background(tcell.ColorBlack).Foreground(tcell.ColorBlack), tcell.RuneBlock
	}
}


func drawTri(s tcell.Screen, to mesh.Tri, logger *log.Logger, theta float64) {
	logger.Printf("Drawing the Tri: %+v", to)
	// style := tcell.StyleDefault.Background(tcell.ColorReset).Foreground(tcell.ColorReset)

	xmax, ymax := s.Size()
	logger.Printf("xmax: %d, ymax: %d", xmax, ymax)
	near := 0.1
	far := 1000.0
	fov := 45.0
	aspectRatio := float64(ymax)/float64(xmax)
	fovRad := 1.0 / math.Tan(fov * 0.5 / 180 * math.Pi)


	matRotZ := [][]float64{
		{math.Cos(theta),math.Sin(theta),0,0},
		{-math.Sin(theta),math.Cos(theta),0,0},
		{0,0,1,0},
		{0,0,0,1},
	}

	matRotX := [][]float64{
		{1,0,0,0},
		{0,math.Cos(theta*0.5),math.Sin(theta*0.5),0},
		{0,-math.Sin(theta*0.5),math.Cos(theta*0.5),0},
		{0,0,0,1},
	}

	// matProj := make([][]float64, 0)
	matProj := [][]float64{
		{0,0,0,0},
		{0,0,0,0},
		{0,0,0,0},
		{0,0,0,0},
	}

	logger.Printf("DEBUG1")
	matProj[0][0] = aspectRatio * fovRad
	logger.Printf("DEBUG2")
	matProj[1][1] = fovRad
	logger.Printf("DEBUG3")
	matProj[2][2] = far / (far - near)
	logger.Printf("DEBUG4")
	matProj[3][2] = (-far * near) / (far - near)
	logger.Printf("DEBUG5")
	matProj[2][3] = 1.0
	logger.Printf("DEBUG6")
	matProj[3][3] = 0.0
	logger.Printf("DEBUG7")

	var tz mesh.Tri
	tz.P1 = multiplyMatrixVector(to.P1, matRotZ, logger)
	tz.P2 = multiplyMatrixVector(to.P2, matRotZ, logger)
	tz.P3 = multiplyMatrixVector(to.P3, matRotZ, logger)
	var tzx mesh.Tri
	tzx.P1 = multiplyMatrixVector(tz.P1, matRotX, logger)
	tzx.P2 = multiplyMatrixVector(tz.P2, matRotX, logger)
	tzx.P3 = multiplyMatrixVector(tz.P3, matRotX, logger)

	var t mesh.Tri
	t = tzx

	t.P1.Z += 8.0
	t.P2.Z += 8.0
	t.P3.Z += 8.0

	var normal, line1, line2 mesh.Point
	line1.X = t.P2.X - t.P1.X
	logger.Printf("line1x: %f, tp2x: %f, tp1x: %f", line1.X, t.P2.X, t.P1.X)
	line1.Y = t.P2.Y - t.P1.Y
	logger.Printf("line1y: %f, tp2y: %f, tp1y: %f", line1.Y, t.P2.Y, t.P1.Y)
	line1.Z = t.P2.Z - t.P1.Z
	logger.Printf("line1z: %f, tp2z: %f, tp1z: %f", line1.Z, t.P2.Z, t.P1.Z)

	line2.X = t.P3.X - t.P1.X
	logger.Printf("line2x: %f, tp3x: %f, tp1x: %f", line1.X, t.P2.X, t.P1.X)
	line2.Y = t.P3.Y - t.P1.Y
	logger.Printf("line2y: %f, tp3y: %f, tp1y: %f", line1.Y, t.P2.Y, t.P1.Y)
	line2.Z = t.P3.Z - t.P1.Z
	logger.Printf("line2y: %f, tp3z: %f, tp1z: %f", line1.Z, t.P2.Z, t.P1.Z)

	normal.X = line1.Y * line2.Z - line1.Z * line2.Y
	normal.Y = line1.Z * line2.X - line1.X * line2.Z
	normal.Z = line1.X * line2.Y - line1.Y * line2.X
	logger.Printf("NormalX: %f, NormalY: %f, NormalZ: %f", normal.X, normal.Y, normal.Z)

	var l float64
	l = math.Sqrt(normal.X*normal.X + normal.Y*normal.Y + normal.Z*normal.Z)
	logger.Printf("l: %f", l)
	normal.X = normal.X / l
	normal.Y = normal.Y / l
	normal.Z = normal.Z / l

	// -0 is for the camera position, for when I want to add that in)
	// Normal is becoming NaN instead of being set properly somewhere. Appears to be happening for all directions
	logger.Printf("normalX: %f, tp1x: %f, normalY: %f, tp1y: %f, normalZ: %f, tp1z: %f", normal.X, t.P1.X, normal.Y, t.P1.Y, normal.Z, t.P1.Z)
	shouldRaster := normal.X * (t.P1.X - 0) + normal.Y * (t.P1.Y - 0) + normal.Z * (t.P1.Z - 0)
	// shouldRaster := normal.Z
	// shouldRaster := -1.0
	logger.Printf("Should Raster: %f", shouldRaster)
	if shouldRaster < 0 {
		lightDirection := mesh.Point{X: 0,Y: 0,Z: 0}
		l = math.Sqrt(lightDirection.X * lightDirection.X + lightDirection.Y * lightDirection.Y + lightDirection.Z * lightDirection.Z)
		lightDirection.X = lightDirection.X / l
		lightDirection.Y = lightDirection.Y / l
		lightDirection.Z = lightDirection.Z / l

		// Used to calculate the saturation. Not sure the option in tcell atm
		var dp float64
		dp = normal.X * lightDirection.X + normal.Y * lightDirection.Y + normal.Z * lightDirection.Z
		logger.Printf("Luminance: %d", 13 * int(dp))
		// style, char := getColor(dp)

		var pt mesh.Tri

		pt.P1 = multiplyMatrixVector(t.P1, matProj, logger)
		pt.P2 = multiplyMatrixVector(t.P2, matProj, logger)
		pt.P3 = multiplyMatrixVector(t.P3, matProj, logger)

		pt.P1.X += 1.0
		pt.P1.Y += 1.0
		pt.P2.X += 1.0
		pt.P2.Y += 1.0
		pt.P3.X += 1.0
		pt.P3.Y += 1.0
		logger.Printf("pt: %+v", pt)

		pt.P1.X *= 0.5 * float64(xmax)
		pt.P1.Y *= 0.5 * float64(ymax)
		pt.P2.X *= 0.5 * float64(xmax)
		pt.P2.Y *= 0.5 * float64(ymax)
		pt.P3.X *= 0.5 * float64(xmax)
		pt.P3.Y *= 0.5 * float64(ymax)
		logger.Printf("pt after transform: %+v", pt)

		logger.Printf("p1x: %d, p1y: %d, p2x: %d, p2y: %d, p3x: %d, p3y: %d", int(pt.P1.X), int(pt.P1.Y), int(pt.P2.X), int(pt.P2.Y), int(pt.P3.X), int(pt.P3.Y))

		if int(pt.P1.X) < xmax && int(pt.P2.X) < xmax && int(pt.P3.X) < xmax && int(pt.P1.Y) < ymax && int(pt.P2.Y) < ymax && int(pt.P3.Y) < ymax {
			logger.Println("Triangle should be visible")
		} else {
			return
		}
		// if int(pt.P1.X) >= xmax {
		// 	logger.Printf("pt.P1.X (%d) is outside of visible bounds", int(pt.P1.X))
		// }
		// if int(pt.P1.Y) >= ymax {
		// 	logger.Printf("pt.P1.Y (%d) is outside of visible bounds", int(pt.P1.Y))
		// }
		// if int(pt.P2.X) >= xmax {
		// 	logger.Printf("pt.P2.X (%d) is outside of visible bounds", int(pt.P2.X))
		// }
		// if int(pt.P2.Y) >= ymax {
		// 	logger.Printf("pt.P2.Y (%d) is outside of visible bounds", int(pt.P2.Y))
		// }
		// if int(pt.P3.X) >= xmax {
		// 	logger.Printf("pt.P3.X (%d) is outside of visible bounds", int(pt.P3.X))
		// }
		// if int(pt.P3.Y) >= ymax {
		// 	logger.Printf("pt.P3.Y (%d) is outside of visible bounds", int(pt.P3.X))
		// }

		// s.SetContent(int(pt.P1.X), int(pt.P1.Y), char, nil, style)
		drawLine(pt.P1, pt.P2, s, logger)
		drawLine(pt.P2, pt.P3, s, logger)
		drawLine(pt.P3, pt.P1, s, logger)
		// s.SetContent(int(pt.P2.X), int(pt.P2.Y), char, nil, style)
		// s.SetContent(int(pt.P2.X), int(pt.P3.Y), char, nil, style)
	}

	// s.SetContent(int(t.P1.X) * 100 , int(t.P1.Y) * 100, tcell.RuneCkBoard, nil, style)
	// s.SetContent(int(t.P2.X) * 100 , int(t.P2.Y) * 100, tcell.RuneCkBoard, nil, style)
	// s.SetContent(int(t.P3.X) * 100 , int(t.P3.Y) * 100, tcell.RuneCkBoard, nil, style)

	// for r := range xmax - 1{
	// 	// logger.Printf("r: %d", r)
	// 	for c := range ymax - 1 {
	// 		// logger.Printf("c: %d", c)
	// 		p := mesh.Point{X: float64(r), Y: float64(c), Z: 0}
	// 		pit := pointInTri(p, t)
	// 		if pit {
	// 			s.SetContent(int(p.X), int(p.Y), tcell.RuneBlock, nil, style)
	// 		// } else {
	// 		// 	s.SetContent(p.x, p.y, tcell.RuneBlock, nil, tcell.StyleDefault.Background(tcell.ColorReset).Foreground(tcell.ColorRed))
	// 		}
	// 		// s.SetContent(r, c, tcell.RuneDiamond, nil, style)
	// 	}
	// }
}

func pointInTri(s mesh.Point, t mesh.Tri) bool {
	a := t.P1
	b := t.P2
	c := t.P3

	as_x := s.X - a.X;
	as_y := s.Y - a.Y

	s_ab := (b.X - a.X) * as_y - (b.Y - a.Y) * as_x > 0

	if ((c.X - a.X) * as_y - (c.Y - a.Y) * as_x > 0 == s_ab) {
		return false
	}
	if ((c.X - b.X) * (s.Y - b.Y) - (c.Y - b.Y) * (s.X - b.X) > 0 != s_ab) {
		return false
	}
	return true
}

func main() {
	fmt.Println("Running...")
	logger := getLogger("gender.log")
	var m mesh.Mesh
	mesh.LoadFromObjectFile("videoship.obj", &m, logger)
	// t1 := mesh.Tri{
	// 		P1: mesh.Point{X: 10, Y: 10, Z: 10},
	// 		P2: mesh.Point{X: 20, Y: 10, Z: 10},
	// 		P3: mesh.Point{X: 10, Y: 20, Z: 10},
	// 		}
	// t2 := mesh.Tri{
	// 		P1: mesh.Point{X: 10, Y: 20, Z: 10},
	// 		P2: mesh.Point{X: 20, Y: 10, Z: 10},
	// 		P3: mesh.Point{X: 20, Y: 20, Z: 10},
	// 		}
	// tm := make([]mesh.Tri, 2)
	// tm = append(tm, t1)
	// tm = append(tm, t2)
	//
	// m := mesh.Mesh{
	// 	Tris: tm,
	// }
	// logger.Printf("Size of obj: %+v", len(m.Tris))
	render(m, logger)
	fmt.Println("End run")
}

func render(m mesh.Mesh, logger *log.Logger) {
	// defStyle := tcell.StyleDefault.Background(tcell.ColorReset).Foreground(tcell.ColorReset)
	// logger := getLogger("gender.log")
	s, err := tcell.NewScreen();
	if err != nil {
		logger.Fatal(err)
	}
	if err := s.Init(); err != nil {
		logger.Fatalf("%+v", err)
	} else {
		logger.Println("Successfully started tcell")
	}
	s.SetStyle(tcell.StyleDefault.Background(tcell.ColorReset).Foreground(tcell.ColorReset))
	// s.EnableMouse()
	// s.EnablePaste()
	s.Clear()

	frameCounter := 0
	xmax, ymax := s.Size()
	logger.Printf("xmax: %d, ymax: %d", xmax, ymax)

	evCh := make(chan tcell.Event)
	quitCh := make(chan struct{})

	theta := 0.0

	go s.ChannelEvents(evCh, quitCh)

loop:

	for {
		s.Clear()
		logger.Printf("Frame: %d", frameCounter)
		logger.Printf("Debug2")

		select {
		case event := <-evCh:
			if event, ok := event.(*tcell.EventKey); ok {
				if event.Key() == tcell.KeyEscape || event.Key() == tcell.KeyCtrlC {
					close(quitCh)
					break loop
				}
			}
			break
		default:
		}

		// var m mesh.Mesh
		theta += 1.0 * float64(frameCounter)

		logger.Printf("Size of obj: %+v", len(m.Tris))
		for _, t := range(m.Tris) {
			drawTri(s, t, logger, theta)
		}

		frameCounter++
		displayFrameNumber(frameCounter, s)
		s.Show()
		time.Sleep(100 * time.Millisecond)
		// }
		s.Show()
	}
	s.Fini()
}

func displayFrameNumber(frameCounter int, s tcell.Screen) {
	chars := []rune(fmt.Sprintf("%d", frameCounter))
	for i, c := range chars {
		s.SetContent(i, 1, c, nil, tcell.StyleDefault.Background(tcell.ColorReset).Foreground(tcell.ColorReset))
	}
}

func getLogger(filename string) *log.Logger {
    logfile, err := os.OpenFile(filename, os.O_TRUNC | os.O_CREATE | os.O_WRONLY, 0666)
    if err != nil {
        fmt.Println("Error opening log file", err.Error())
        panic(err)
    }
    return log.New(logfile, "[gender] ", log.LstdFlags)
}
