package mesh

import (
	"os"
	"log"
	"bufio"
	"strings"
	"strconv"
)

type Point struct {
	X, Y, Z float64
}

type Tri struct {
	P1, P2, P3 Point
}

type Mesh struct {
	Tris []Tri
}

func LoadFromObjectFile(filename string, m *Mesh, logger *log.Logger) {
	// logger.Printf("DEBUG1")
    meshfile, err := os.Open(filename)
	// logger.Printf("DEBUG2")
	if err != nil {
		// logger.Printf("%v", err)
		log.Fatal(err)
	}
	defer meshfile.Close()

	var verts []Point

	scanner := bufio.NewScanner(meshfile)
	// logger.Printf("DEBUG3")
	for scanner.Scan() {
		line := scanner.Text()
		// logger.Printf("DEBUG4")
		if line[0] == 'v' {
			// logger.Printf("DEBUG5")
			var v Point
			temp := strings.Split(line, " ")
			logger.Printf("vline: %+v", temp)
			// logger.Printf("DEBUG6")
			v.X, err = strconv.ParseFloat(temp[1], 64)
			if err != nil {
				// logger.Printf("DEBUG7")
				logger.Printf("%v", err)
			}
			v.Y, err = strconv.ParseFloat(temp[2], 64)
			if err != nil {
				// logger.Printf("DEBUG8")
				logger.Printf("%v", err)
			}
			v.Z, err = strconv.ParseFloat(temp[3], 64)
			if err != nil {
				// logger.Printf("DEBUG9")
				logger.Printf("%v", err)
			}
			verts = append(verts, v)
		}

		if line[0] == 'f' {
			// logger.Printf("DEBUG10")
			temp := strings.Split(line, " ")
			logger.Printf("fline: %+v", temp)
			// logger.Printf("DEBUG11")
			var f [3]int64
			f[0], err = strconv.ParseInt(temp[1], 10, 32)
			if err != nil {
				// logger.Printf("DEBUG12")
				logger.Printf("%v", err)
			}
			f[1], err = strconv.ParseInt(temp[2], 10, 32)
			if err != nil {
				// logger.Printf("DEBUG13")
				logger.Printf("%v", err)
			}
			f[2], err = strconv.ParseInt(temp[3], 10, 32)
			if err != nil {
				// logger.Printf("DEBUG14")
				logger.Printf("%v", err)
			}

			// logger.Printf("DEBUG15")
			// logger.Printf("f[0]: %d, f[1]: %d, f[2]: %d", f[0], f[1], f[2])
			// logger.Printf("verts: %+v", verts)
			m.Tris = append(m.Tris, Tri{
				verts[f[0] - 1],
				verts[f[1] - 1],
				verts[f[2] - 1],
			})
			// logger.Printf("DEBUG16")
		}
	}

	logger.Printf("Object Loaded: %+v", m.Tris)

	if err := scanner.Err(); err != nil {
		// logger.Printf("DEBUG16")
		logger.Printf("%v", err)
	}
}
