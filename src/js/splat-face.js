  import layoutHtml from '../html/splat-face.html?html'
  import { SplatBase } from './splat-base'

  export class SplatFace extends SplatBase {
      constructor(state) {
        super()
        this.state = {...state}
        this.view.innerHTML = layoutHtml

        this.options = { maxFaces: 1, refineLandmarks: false, flipped: false }
        this.faces = []

        this.loadML5()
      }

      async init(){
        if(!window.ml5) {
          setTimeout(()=>this.init(), 1000)
          return
        }
        console.log('ML5 Loaded')
        this.faceMesh = ml5.faceMesh(this.options);


        // this.video = await this.startWebcamCapture()
        // 
        // this.video.addEventListener("loadedmetadata", () => {
        //   console.log('Start detection')
        //   this.faceMesh.detectStart(this.video, (r)=>this.gotFace(r))        
        // });


        


        //

        this.view.querySelector('.Camera').addEventListener('click', ()=>{
          this.canvas = this.captureWebcamToCanvas();
        })

        this.view.querySelector('.Capture').addEventListener('click', ()=>{


          this.triangles = this.faceMesh.getTriangles();
          this.faceMesh.detectStart(this.canvas, (r)=>this.gotFaces(r)) 
          // faceMesh.detectStop()
        })
        
      }
      

      /*
        Results:
        [
          {
            box: { width, height, xMax, xMin, yMax, yMin },
            keypoints: [{ x, y, z, name }, ... ],
            faceOval: { x, y, width, height, centerX, centerY, keypoints: [{ x, y, z }, ... ]},
            leftEye: { x, y, width, height, centerX, centerY, keypoints: [{ x, y, z }, ... ]},
            ...
          },
          ...
        ]
      */
      gotFaces(results){
        this.faces = results

        // console.log('Faces found!!', results)
        if(!this.log) {
          this.log = true 
          //console.log('Faces found!!', JSON.stringify(results))
          console.log('Faces found!!', JSON.stringify(this.triangles))
        }
      }

      loadML5(){
        let scr = document.querySelector('#ml5_script')
        //if(scr) scr.parentNode.removeChild(scr)
        if(!scr) {
          scr = document.createElement('script')
          scr.addEventListener('loaded', ()=>{
            this.init()
          })
          scr.id = 'ml5_script'
          //scr.src = 'https://unpkg.com/ml5@1.2.1/dist/ml5.js'
          scr.src = 'https://unpkg.com/ml5@1/dist/ml5.js'
          document.body.appendChild(scr)
        }else{
          this.init()
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
          //videoElement.style.display = 'none'
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

    captureFrame(videoElement, format = 'canvas') {
      // Create a canvas element
      const canvas = document.createElement('canvas');
      canvas.width = videoElement.videoWidth;  // Set canvas width to video width
      canvas.height = videoElement.videoHeight; // Set canvas height to video height
  
      // Draw the current video frame onto the canvas
      const context = canvas.getContext('2d');
      context.drawImage(videoElement, 0, 0, canvas.width, canvas.height);
  
      // Return the frame based on the specified format
      if (format === 'canvas') {
          return canvas; // Return the canvas element
      } else if (format === 'dataURL') {
          return canvas.toDataURL('image/png'); // Return the frame as a data URL (PNG format)
      } else {
          throw new Error('Invalid format specified. Use "canvas" or "dataURL".');
      }
    }


    captureWebcamToCanvas() {
      // Create a video element
      const videoElement = document.createElement('video');
      videoElement.autoplay = true; // Ensure the video autoplays
      this.video = videoElement
  
      // Create a canvas element
      const canvasElement = document.createElement('canvas');
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

  }


  
  customElements.define('splat-face', SplatFace)
  