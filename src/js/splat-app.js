  import layoutHtml from '../html/splat-app.html?html'
  import { SplatBase } from './splat-base'

  export class SplatApp extends SplatBase {
      constructor(state) {
        super()
        this.state = {...state}
        this.view.innerHTML = layoutHtml
      }
  }
  customElements.define('splat-app', SplatApp)
  