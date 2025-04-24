import BendisRouter from 'bendis/BendisRouter'
  export class SplatRouter extends BendisRouter{
      constructor(){
          super('splat-')
      }
  }
  customElements.define('splat-route', SplatRouter)