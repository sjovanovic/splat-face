  import layoutHtml from '../html/splat-view.html?html'
  import { SplatBase } from './splat-base'
  import * as SPLAT from 'gsplat';
  import { genSplatFromTrianglesAnd2dContext, findNearestPointOnRay } from './splatgen'

  export class SplatView extends SplatBase {
      constructor(state) {
        super()
        this.state = {
          sliders:[

            {
              name:'RX', value:0, min:-1, max:1, step:0.01, 
              input:(ev)=>{ 
                if(!this.splats.customRotations) this.splats.customRotations = {x:0, y:0, z:0, w:0}
                this.splats.customRotations.x = parseFloat(ev.target.value)
                console.log(ev.target.value)  
              } 
            },

            {
              name:'RY', value:0, min:-1, max:1, step:0.01, 
              input:(ev)=>{ 
                if(!this.splats.customRotations) this.splats.customRotations =  {x:0, y:0, z:0, w:0}
                this.splats.customRotations.y = parseFloat(ev.target.value)
                console.log(ev.target.value)  
              } 
            },

            {
              name:'RZ', value:0, min:-1, max:1, step:0.01, 
              input:(ev)=>{ 
                if(!this.splats.customRotations) this.splats.customRotations =  {x:0, y:0, z:0, w:0}
                this.splats.customRotations.z = parseFloat(ev.target.value)
                console.log(ev.target.value)  
              } 
            },

            {
              name:'RW', value:0, min:-1, max:1, step:0.01, 
              input:(ev)=>{ 
                if(!this.splats.customRotations) this.splats.customRotations =  {x:0, y:0, z:0, w:0}
                this.splats.customRotations.w = ev.target.value
                console.log(ev.target.value)  
              } 
            },

            {
              name:'SX', value:0.002, min:0.0001, max:0.01, step:0.0002, 
              input:(ev)=>{ 
                if(!this.splats.customScales) this.splats.customScales =  {x:0, y:0, z:0}
                this.splats.customScales.x = ev.target.value
                console.log(ev.target.value)  
              } 
            },

            {
              name:'SY',value:0.002, min:0.0001, max:0.01, step:0.0002, 
              input:(ev)=>{ 
                if(!this.splats.customScales) this.splats.customScales =  {x:0, y:0, z:0}
                this.splats.customScales.y = ev.target.value
                console.log(ev.target.value)  
              } 
            },

            {
              name:'SZ', value:0.002, min:0.0001, max:0.01, step:0.0002, 
              input:(ev)=>{ 
                if(!this.splats.customScales) this.splats.customScales =  {x:0, y:0, z:0}
                this.splats.customScales.z = ev.target.value
                console.log(ev.target.value)  
              } 
            },

          ],
          ...state
        }
        this.view.innerHTML = layoutHtml
        
        this.slidersInit()
      }

      init(){
        if(this.inited) return 
        this.inited = true
        console.log('Initing')
        

        // Set up the scene

        this.scene = new SPLAT.Scene();
        this.camera = new SPLAT.Camera();
        this.renderer = new SPLAT.WebGLRenderer(this.view.querySelector('.Canvas'));
        this.controls = new SPLAT.OrbitControls(this.camera, this.renderer.canvas);

        this.testRay()


        //SPLAT.Loader.LoadAsync('./bonsai-7k-mini.splat', this.scene, () => {});
        // let splat = this.splat = this.genRandomSplat(1000)
        // this.scene.addObject(splat)

        // genSplatFromFaceMesh


        // clearTimeout(window.__iml5)
        // window.__iml5 = setTimeout(()=>{



        //   let firstRun = true
        //   this.startFaceMesh(()=>{
        //     if(firstRun) {
        //       firstRun = false 
        //       let splat = this.genSplatFromFaceMesh()

        //       console.log('Splat created', splat)

        //       this.scene.addObject(splat)
        //     }
        //   })
        // }, 1000)


        // this.view.querySelector('.Load').addEventListener('click', ()=>{
        //   this.loadML5(()=>{
        //     console.log('Loaded ML5')
        //   })
        // })

        this.view.querySelector('.Start').addEventListener('click', ()=>{
          let firstRun = true
          this.startFaceMesh(()=>{
            // if(firstRun) {
            //   firstRun = false 
            //   let splat = this.genSplatFromFaceMesh()
            //   console.log('Splat created', splat)
            //   this.scene.addObject(splat)
            // }
          })
        })

        this.view.querySelector('.Detect').addEventListener('click', ()=>{
          let firstRun = true
          this.triangles = this.faceMesh.getTriangles();
          this.faceMesh.detectStart(this.canvas, (results)=>{
            this.faces = results
            


            if(firstRun) {
              firstRun = false 
              let splat = this.genSplatFromFaceMesh()
              console.log('Splat created', splat)
              this.scene.addObject(splat)
            }else{

              this.genSplatFromFaceMesh()

              this.splats.changed = true
              
              
            }



          })    
        })

        

        this.view.querySelector('.Stop').addEventListener('click', ()=>{
          this.stopFaceMesh()
        })

        
        
      }

      updateSplat(){


        // let cnt = this.splats.vertexCount

        // for(let i=0;i<cnt;i=i+3){
        //   this.splats.positions[i] = this.splats.positions[i] + Math.sin(Date.now() * 0.01)
        //   this.splats.positions[i+1] = this.splats.positions[i+1] + Math.sin(Date.now() * 0.01)
        //   this.splats.positions[i+2] = this.splats.positions[i+2] + Math.sin(Date.now() * 0.01)

        //   // this.splats.positions[i] = this.splats.positions[i] + (Math.random() - 0.5) * 0.1;
        //   // this.splats.positions[i+1] = this.splats.positions[i+1] + (Math.random() - 0.5) * 0.1;
        //   // this.splats.positions[i+2] = this.splats.positions[i+2] + (Math.random() - 0.5) * 0.1;

        // }
        // this.splats.changed = true
        // this.splat.dispatchEvent(this.splat._changeEvent);

      }

      

      genRandomSplat(cnt=10){
        // Define some splats

        // positions
        let position = new Float32Array(cnt * 3), siz = 10, hsiz = siz * 0.5
        for(let i=0;i<cnt;i=i+3){
          position[i] = Math.random() * siz - hsiz
          position[i+1] = Math.random() * siz - hsiz
          position[i+2] = Math.random() * siz - hsiz
        }

        // colors
        let colors = new Uint8Array(cnt * 4)
        for(let i=0;i<cnt;i=i+4){
          colors[i] = parseInt(Math.random() * 255)
          colors[i+1] = parseInt(Math.random() * 255)
          colors[i+2] = parseInt(Math.random() * 255)
          colors[i+3] = parseInt(Math.random() * 255)
        }


        // scales
        let scales = new Float32Array(cnt * 3), scl = 0.1
        for(let i=0;i<cnt;i=i+3){
          scales[i] = Math.random() * scl
          scales[i+1] = Math.random() * scl
          scales[i+2] = Math.random() * scl
        }

        // rotations
        let rotations = new Float32Array(cnt * 4), rto = 2
        for(let i=0;i<cnt;i=i+4){
          rotations[i] = parseInt(Math.random() * rto)
          rotations[i+1] = parseInt(Math.random() * rto)
          rotations[i+2] = parseInt(Math.random() * rto)
          rotations[i+3] = parseInt(Math.random() * rto)
        }

        this.splats = new SPLAT.SplatData(cnt, position, rotations, scales, colors)

        let splatObj = new SPLAT.Splat(this.splats)
        return splatObj
      }


      // genSplat(obj){
      //   const RowLength = 3 * 4 + 3 * 4 + 4 + 4;
      //   const data = new Uint8Array(this.vertexCount * RowLength);

      //   const f_buffer = new Float32Array(data.buffer);
      //   const u_buffer = new Uint8Array(data.buffer);

      //   for (let i = 0; i < this.vertexCount; i++) {
      //       f_buffer[8 * i + 0] = this.positions[3 * i + 0];
      //       f_buffer[8 * i + 1] = this.positions[3 * i + 1];
      //       f_buffer[8 * i + 2] = this.positions[3 * i + 2];

      //       u_buffer[32 * i + 24 + 0] = this.colors[4 * i + 0];
      //       u_buffer[32 * i + 24 + 1] = this.colors[4 * i + 1];
      //       u_buffer[32 * i + 24 + 2] = this.colors[4 * i + 2];
      //       u_buffer[32 * i + 24 + 3] = this.colors[4 * i + 3];

      //       f_buffer[8 * i + 3 + 0] = this.scales[3 * i + 0];
      //       f_buffer[8 * i + 3 + 1] = this.scales[3 * i + 1];
      //       f_buffer[8 * i + 3 + 2] = this.scales[3 * i + 2];

      //       u_buffer[32 * i + 28 + 0] = (this.rotations[4 * i + 0] * 128 + 128) & 0xff;
      //       u_buffer[32 * i + 28 + 1] = (this.rotations[4 * i + 1] * 128 + 128) & 0xff;
      //       u_buffer[32 * i + 28 + 2] = (this.rotations[4 * i + 2] * 128 + 128) & 0xff;
      //       u_buffer[32 * i + 28 + 3] = (this.rotations[4 * i + 3] * 128 + 128) & 0xff;
      //   }

      //   return data;
      // }

      animate(){
        const frame = () => {

          setTimeout(()=>this.updateSplat(), 1)

          this.controls.update();
          this.renderer.render(this.scene, this.camera);
          if(!this.running) return

          requestAnimationFrame(frame);
        };
        requestAnimationFrame(frame);
      }

      connectedCallback(){
        // let r = this.closestComponentByName('splat-route')
        // console.log('r', r)
        this.running = true
        this.init()
        this.animate()
      }

      disconnectedCallback(){
        this.running = false
      }


      /*
        Face Mesh
      */
      loadML5(cb=function(){}){
        if(window.ml5){ cb(window.ml5) }
        let scr = document.querySelector('#ml5_script')
        if(scr) scr.parentNode.removeChild(scr)
        if(!scr) {
          scr = document.createElement('script')
          scr.addEventListener('load', ()=>{
            this.whenMl5Ready(cb)
          })
          scr.id = 'ml5_script'
          //scr.src = 'https://unpkg.com/ml5@1.2.1/dist/ml5.js'
          scr.src = 'https://unpkg.com/ml5@1/dist/ml5.min.js'
          document.body.appendChild(scr)
        }else{
          this.whenMl5Ready(cb)
        }
      }
      whenMl5Ready(cb=function(){}, tries = 60, ms=500){
        if(window.ml5){
          console.log('Ready')
          cb(window.ml5)
        } else if(tries > 0) {
          console.log('Timeout Ready')
          setTimeout(()=>this.whenMl5Ready(cb, tries-1, ms), ms)
        }
      }
      startWebcamCapture(videoWidth = 640, videoHeight = 480) {
        // Create a video element

        let videoElement = this.view.querySelector('#ml5_video');
        if(!videoElement) {
          videoElement = document.createElement('video');
          videoElement.width = videoWidth;  // Set video width
          videoElement.height = videoHeight; // Set video height
          videoElement.autoplay = true;     // Ensure the video autoplays
          videoElement.id = 'ml5_video'
          videoElement.style.display = 'none'
          this.view.appendChild(videoElement); // Add the video element to the DOM
        }
    
        // Check if the browser supports mediaDevices and getUserMedia
        if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
            // Request access to the webcam
            navigator.mediaDevices.getUserMedia({ video: true })
                .then((stream) => {
                    // Set the video source to the webcam stream
                    videoElement.srcObject = stream;
                })
                .catch((error) => {
                    console.error('Error accessing the webcam:', error);
                });
        } else {
            console.error('getUserMedia is not supported in this browser.');
        }
    
        return videoElement; // Return the video element for further use
      }
      startFaceMesh(cb){
        this.fmOpts = {width: 640, height: 480} 
        this.canvas = this.captureWebcamToCanvas(this.fmOpts.width, this.fmOpts.height);

        this.loadML5((ml5)=>{
          
          this.faceMesh = ml5.faceMesh({ maxFaces: 1, refineLandmarks: false, flipped: false })
          

          
          // this.triangles = this.faceMesh.getTriangles();
          // this.faceMesh.detectStart(this.canvas, (results)=>{

          //   console.log('.')

          //   this.faces = results
          //   cb(results)
          // })      
          
        })
      }




      stopFaceMesh(){
        if(this.faceMesh) this.faceMesh.detectStop()
        if(this.video) this.video.parentNode.removeChild(this.video)
        if(this.triangles) this.triangles = []
      }
      norm(value, min, max){
        return (value - min) / (max - min)
      }

      genSplatFromFaceMesh(){
        if(!this.faces || !this.faces.length || !this.faces[0].keypoints || !this.faces[0].keypoints.length) return
        let keypoints = this.faces[0].keypoints
        let indices = this.triangles

        let ctx = this.canvas.getContext('2d');

        if(this.splat) {
          genSplatFromTrianglesAnd2dContext(keypoints, indices, ctx, this.splats)
          this.splats.changed = true
          this.splat.dispatchEvent(this.splat._changeEvent);
          return this.splat
        }
        
        let { positions, colors, scales, rotations } = genSplatFromTrianglesAnd2dContext(keypoints, indices, ctx)
        let cnt = Math.round(positions.length / 3)

        this.splats = new SPLAT.SplatData(cnt, positions, rotations, scales, colors)
        this.splat = new SPLAT.Splat(this.splats)
        return this.splat

      }

      genSplatFromFaceMesh_old2(){

        let min = 0, max = 500

        if(!this.faces || !this.faces.length || !this.faces[0].keypoints || !this.faces[0].keypoints.length) return
        let face = this.faces[0].keypoints
        if(!this.splat) {
          
          let cnt = face.length

          let tsplat = {
            positions: new Float32Array(cnt * 3),
            colors: new Uint8Array(cnt * 4),
            scales: new Float32Array(cnt * 3),
            rotations: new Float32Array(cnt * 4)
          }

          let {positions, colors, scales, rotations} = tsplat

          let points = face.map(t => {
            return [
              this.norm(t.x, min, max),
              this.norm(t.y, min, max),
              this.norm(t.z, min, max)
            ]
          })

          for(let i=0;i<cnt;i++){
            positions[i*3] = points[i][0]
            positions[i*3+1] = points[i][1]
            positions[i*3+2] = points[i][2]
          }


          const ctx = this.canvas.getContext('2d');

        //   const pixelData = ctx.getImageData(x, y, 1, 1).data;

        // // Convert the pixel data to an array of RGBA values
        // return Array.from(pixelData); // [r, g, b, a]

          let color
          for(let i=0;i<cnt;i++){
            // color = ctx.getImageData(face[i].x, face[i].y, 1, 1).data;
            // colors[i*4] = color[0]
            // colors[i*4+1] = color[1]
            // colors[i*4+2] = color[2]
            // colors[i*4+3] = color[3]

            colors[i*4] = parseInt(Math.random() * 255)
            colors[i*4+1] = parseInt(Math.random() * 255)
            colors[i*4+2] = parseInt(Math.random() * 255)
            colors[i*4+3] = parseInt(Math.random() * 255)
          }

          // scales
          let scl = 0.003
          for(let i=0;i<cnt;i++){
            scales[i*3] = Math.random() * scl
            scales[i*3+1] = Math.random() * scl
            scales[i*3+2] = Math.random() * scl
            // scales[i*3] = scl
            // scales[i*3+1] = scl
            // scales[i*3+2] = scl
          }


          // idx to triangle
          let idxTri = {}
          this.triangles.forEach(t => {
            let tri = [points[t[0]], points[t[1]], points[t[2]]]
            idxTri[t[0]] = tri
            idxTri[t[1]] = tri
            idxTri[t[2]] = tri
          })
          

          // rotations
          for(let i=0;i<cnt;i++){
            let tri = idxTri[i]
            let quaternion = this.getFaceNormalQuaternion(tri[0], tri[1], tri[2])

            rotations[i*4] = quaternion[0]
            rotations[i*4+1] = quaternion[1]
            rotations[i*4+2] = quaternion[2]
            rotations[i*4+3] = quaternion[3]
          }
    
          this.splats = new SPLAT.SplatData(cnt, positions, rotations, scales, colors)
          let splatObj = new SPLAT.Splat(this.splats)
          this.splat = splatObj
          return splatObj



        }else{

          let cnt = this.splats.vertexCount

          let points = face.map(t => {
            return [
              this.norm(t.x, min, max),
              this.norm(t.y, min, max),
              this.norm(t.z, min, max)
            ]
          })

          for(let i=0;i<cnt;i++){
            this.splats.positions[i*3] = points[i][0]
            this.splats.positions[i*3+1] = points[i][1]
            this.splats.positions[i*3+2] = points[i][2]
          }

          this.splats.changed = true
          this.splat.dispatchEvent(this.splat._changeEvent);

          return this.splat
        }
      }



      /**
       * Gets the color at the specified canvas coordinates.
       * @param {HTMLCanvasElement} canvas - The canvas element.
       * @param {number} x - The x-coordinate.
       * @param {number} y - The y-coordinate.
       * @returns {number[]} - The color as an array of RGBA values [r, g, b, a].
       */
      getColorAtCanvasCoordinates(canvas, x, y) {
        // Get the 2D rendering context of the canvas
        const ctx = canvas.getContext('2d');

        // Get the pixel data at the specified coordinates
        const pixelData = ctx.getImageData(x, y, 1, 1).data;

        // Convert the pixel data to an array of RGBA values
        return Array.from(pixelData); // [r, g, b, a]
      }



      genSplatFromFaceMesh_old(opts={update:false}){


        if(this.splat) {

          let cnt = this.splats.vertexCount

          //this.triangles = this.faceMesh.getTriangles();
          let triangles = this.triangles

          // normalize between 0 and 1
          let ntriangles = triangles.map(t => {
            return [
              this.norm(t[0], -1000, 1000),
              this.norm(t[1], -1000, 1000),
              this.norm(t[2], -1000, 1000)
            ]
          })

          //let ntriangles = triangles

          // calculate centroids
          let centroids = []
          for(let i=0;i<cnt;i=i+3){
            centroids.push(this.getTriangleCentroid(ntriangles[i], ntriangles[i+1], ntriangles[i+2]))
          }

          
          
          // positions
          for(let i=0;i<centroids.length;i++){
            if(i === 0) console.log(centroids[i][0])
            this.splats.positions[i*3] = centroids[i][0]
            this.splats.positions[i*3+1] = centroids[i][1]
            this.splats.positions[i*3+2] = centroids[i][2]
          }




          this.splats.changed = true
          this.splat.dispatchEvent(this.splat._changeEvent);

          return this.splat
        }

        //if(!this.triangles) return
        let {width, height} = this.fmOpts

        if(!this.faces || !this.faces.length || !this.faces[0].keypoints || !this.faces[0].keypoints.length) return
        let face = this.faces[0].keypoints

        let triangles = this.triangles
        let cnt = triangles.length / 3
        if(!this.tsplat) this.tsplat = {
          positions: new Float32Array(cnt * 3),
          colors: new Uint8Array(cnt * 4),
          scales: new Float32Array(cnt * 3),
          rotations: new Float32Array(cnt * 4)
        }

        // normalize between 0 and 1

        console.log('cnt', cnt, 'face', face.length)

        let ntriangles = triangles.map(t => {
          return [
            this.norm(t[0], -1000, 1000),
            this.norm(t[1], -1000, 1000),
            this.norm(t[2], -1000, 1000)
          ]
        })

        // let ntriangles = face.map(t => {
        //   return [
        //     this.norm(t.x, -1000, 1000),
        //     this.norm(t.y, -1000, 1000),
        //     this.norm(t.z, -1000, 1000)
        //   ]
        // })

        

        // calculate centroids
        let centroids = []
        for(let i=0;i<ntriangles.length;i=i+3){
          centroids.push(this.getTriangleCentroid(ntriangles[i], ntriangles[i+1], ntriangles[i+2]))
        }

        let {positions, colors, scales, rotations } = this.tsplat
        
        // positions
        for(let i=0;i<cnt;i++){
          positions[i*3] = centroids[i][0]
          positions[i*3+1] = centroids[i][1]
          positions[i*3+2] = centroids[i][2]
        }

        // for(let i=0;i<cnt;i = i + 3){
        //   positions[i*3] = ntriangles[i][0]
        //   positions[i*3+1] = ntriangles[i][1]
        //   positions[i*3+2] = ntriangles[i][2]
        // }


        // colors
        for(let i=0;i<cnt;i++){
          colors[i*4] = parseInt(Math.random() * 255)
          colors[i*4+1] = parseInt(Math.random() * 255)
          colors[i*4+2] = parseInt(Math.random() * 255)
          colors[i*4+3] = parseInt(Math.random() * 255)
        }


        // scales
        let scl = 0.01
        for(let i=0;i<cnt;i++){
          // scales[i*3] = Math.random() * scl
          // scales[i*3+1] = Math.random() * scl
          // scales[i*3+2] = Math.random() * scl
          scales[i*3] = scl
          scales[i*3+1] = scl
          scales[i*3+2] = scl
        }

        // rotations
        for(let i=0;i<cnt;i++){
          let quaternion = this.getFaceNormalQuaternion(ntriangles[i], ntriangles[i+1], ntriangles[i+2])
          rotations[i] = quaternion[0]
          rotations[i+1] = quaternion[1]
          rotations[i+2] = quaternion[2]
          rotations[i+3] = quaternion[3]
        }
  
        this.splats = new SPLAT.SplatData(cnt, positions, rotations, scales, colors)
        let splatObj = new SPLAT.Splat(this.splats)
        this.splat = splatObj
        return splatObj
        
      }

      /**
       * Computes the centroid (center point) of a triangle given its three vertices.
       * @param {number[]} v1 - First vertex of the triangle [x, y, z].
       * @param {number[]} v2 - Second vertex of the triangle [x, y, z].
       * @param {number[]} v3 - Third vertex of the triangle [x, y, z].
       * @returns {number[]} - Centroid as an array of three floats [x, y, z].
       */
      getTriangleCentroid(v1, v2, v3) {
        // Calculate the average of the x, y, and z coordinates
        const x = (v1[0] + v2[0] + v3[0]) / 3;
        const y = (v1[1] + v2[1] + v3[1]) / 3;
        const z = (v1[2] + v2[2] + v3[2]) / 3;

        return [x, y, z];
      }

      /**
       * Computes the quaternion representing the rotation of a triangle's face normal.
       * @param {number[]} v1 - First vertex of the triangle [x, y, z].
       * @param {number[]} v2 - Second vertex of the triangle [x, y, z].
       * @param {number[]} v3 - Third vertex of the triangle [x, y, z].
       * @returns {number[]} - Quaternion as an array of integers [x, y, z, w].
       */
      getFaceNormalQuaternion(v1, v2, v3) {
        // Helper function to subtract two vectors
        function subtractVectors(a, b) {
            return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        }

        // Helper function to compute the cross product of two vectors
        function crossProduct(a, b) {
            return [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            ];
        }

        // Helper function to normalize a vector
        function normalizeVector(v) {
            const length = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            return [v[0] / length, v[1] / length, v[2] / length];
        }

        // Compute two edges of the triangle
        const edge1 = subtractVectors(v2, v1);
        const edge2 = subtractVectors(v3, v1);

        // Compute the face normal using the cross product
        const faceNormal = crossProduct(edge1, edge2);
        const normalizedFaceNormal = normalizeVector(faceNormal);

        // Define the target normal (e.g., Y-axis)
        const targetNormal = [0, 1, 0];

        // Compute the rotation axis using the cross product of the face normal and target normal
        const rotationAxis = crossProduct(normalizedFaceNormal, targetNormal);
        const normalizedRotationAxis = normalizeVector(rotationAxis);

        // Compute the rotation angle using the dot product
        const dotProduct = normalizedFaceNormal[0] * targetNormal[0] +
                          normalizedFaceNormal[1] * targetNormal[1] +
                          normalizedFaceNormal[2] * targetNormal[2];
        const rotationAngle = Math.acos(Math.min(Math.max(dotProduct, -1), 1)); // Clamp to avoid NaN

        // Compute the quaternion components
        const halfAngle = rotationAngle / 2;
        const sinHalfAngle = Math.sin(halfAngle);
        const quaternion = [
            normalizedRotationAxis[0] * sinHalfAngle, // x
            normalizedRotationAxis[1] * sinHalfAngle, // y
            normalizedRotationAxis[2] * sinHalfAngle, // z
            Math.cos(halfAngle)                      // w
        ];

        return quaternion

        // // Scale and round the quaternion components to integers
        // const scale = 1000; // Scale factor for converting to integers
        // const integerQuaternion = quaternion.map(component => Math.round(component * scale));

        // return integerQuaternion;
      }

      captureWebcamToCanvas(width=640, height=480) {
        // Create a video element
        const videoElement = document.createElement('video');
        videoElement.width = width 
        videoElement.height = height
        videoElement.autoplay = true; // Ensure the video autoplays
        this.video = videoElement
    
        // Create a canvas element
        const canvasElement = document.createElement('canvas');
        canvasElement.className = 'VideoCanvas'
        const context = canvasElement.getContext('2d');
        this.view.appendChild(canvasElement); // Add the canvas to the DOM
    
        // Check if the browser supports mediaDevices and getUserMedia
        if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
            // Request access to the webcam
            navigator.mediaDevices.getUserMedia({ video: true })
                .then((stream) => {
                    // Set the video source to the webcam stream
                    videoElement.srcObject = stream;
    
                    // Wait for the video metadata to load to get the video dimensions
                    videoElement.addEventListener('loadedmetadata', () => {
                        // Set canvas dimensions to match the video
                        canvasElement.width = videoElement.videoWidth;
                        canvasElement.height = videoElement.videoHeight;
    
                        // Function to draw video frames onto the canvas
                        function drawFrame() {
                            context.drawImage(videoElement, 0, 0, canvasElement.width, canvasElement.height);
                            requestAnimationFrame(drawFrame); // Continuously draw frames
                        }
    
                        // Start drawing frames
                        drawFrame();
                    });
                })
                .catch((error) => {
                    console.error('Error accessing the webcam:', error);
                });
        } else {
            console.error('getUserMedia is not supported in this browser.');
        }
    
        return canvasElement; // Return the canvas element
      }

      testRay(){
          this.renderer.canvas.addEventListener('click', (ev)=>{
            const rect = this.renderer.canvas.getBoundingClientRect()
            const mouseX = ev.clientX - rect.left
            const mouseY = ev.clientY - rect.top
            const x = (mouseX / rect.width) * 2 - 1;
            const y = -(mouseY / rect.height) * 2 + 1;
            let ray = this.camera.screenPointToRay(x, y)
            const rayOrigin = [this.camera.position.x, this.camera.position.y, this.camera.position.z]; // Ray origin
            const rayDirection = [ray.x, ray.y, ray.z]; // Ray direction
            //console.log('ray', {x, y, rayOrigin, rayDirection})
            let point = findNearestPointOnRay(this.splats.positions, rayOrigin, rayDirection, 0.01)

            if(point){
              this.splats.displacementOrigin = point.point
              this.splats.displacementDirection = this.getDirectionVector(point.point, rayOrigin)
            }else{
              this.splats.displacementOrigin = null
              this.splats.displacementDirection = null
            }

            console.log('result point', point)
          })
      }

      getDirectionVector(origin, destination, normalize = true) {
        // Calculate the direction vector
        const direction = [
            destination[0] - origin[0],
            destination[1] - origin[1],
            destination[2] - origin[2],
        ];
    
        // Normalize the vector if requested
        if (normalize) {
            const length = Math.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2);
            direction[0] /= length;
            direction[1] /= length;
            direction[2] /= length;
        }
    
        return direction;
      }


      slidersInit(){
        this.bind('sliders.*', '.SliderWrapper', Array)
        this.bind('sliders.*', '.Slider', (ctx)=>{
          let {el, val} = ctx
          el.setAttribute('min', (val?.min || 0) + "")
          el.setAttribute('max', (val?.max || 100) + "")
          el.setAttribute('step', (val?.step || 1) + "")
          el.value = val?.value || 0
          el.oninput = val?.input || function(){}

          el.oninput = (ev)=>{
            val.value = parseFloat(ev.target.value)
            if(val?.input) val?.input(ev)
          }
        })
        this.bind('sliders.*.name', '.SliderName') 
        this.bind('sliders.*.value', '.SliderValue') 
        
      }
  }
  customElements.define('splat-view', SplatView)
  
  