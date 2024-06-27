async function myfunc(no){
    let break_flag =0;
    while (1){
        const controller = new AbortController()
        const timeoutId = setTimeout(() => controller.abort(), 1500)
        new Promise((resolve, reject) => {
        return fetch('http://192.168.0.'+no+':5000',
        {signal: controller.signal}
        ).then(response => {

            if (response.status === 200) {
            // let data = await response.text();
            // handle data
            console.log('server active_______________________');
            break_flag=1;
            }
            else{
                reject(new Error('error'));
                console.log('server inactive');
            }
        }, error => {
            // reject(new Error(error.message));
        })
        });
        if(break_flag==0)
        {
            console.log('Inactive');
        }
        else{
            postMessage(true);
            console.log('Active');
            break;
        }
        
        await new Promise(r => setTimeout(r, 1000));
    }

}

var parameters = {}
location.search.slice(1).split("&").forEach( function(key_value) { var kv = key_value.split("="); parameters[kv[0]] = kv[1]; })

var ip_addr = parameters['ip_addr'];
myfunc(ip_addr);

