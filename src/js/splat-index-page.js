  import layoutHtml from '../html/splat-index-page.html?html'
  import { SplatBase } from './splat-base'

  export class SplatIndexPage extends SplatBase {
      constructor(state) {
        super()
        this.state = {...state}
        this.view.innerHTML = layoutHtml
      }
  }
  customElements.define('splat-index-page', SplatIndexPage)
  