import Bendis from 'bendis/Bendis'
  export class SplatBase extends Bendis {
    closestComponentByName(name){
        let el = this, c = null
        for(let i=0;i<100; i++){
            c = this.closestComponent(el)
            if(!c || c == el) return null
            console.log(c.nodeName.toLowerCase())
            if(c.nodeName.toLowerCase() == name) {
                return c
            }
            el = c
        }
        return null
    }

    closestComponent(el = this){
        return el.getRootNode().host
    }
  }