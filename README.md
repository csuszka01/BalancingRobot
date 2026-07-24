# BalancingRobot

teszt1

## RT-FaaS scheduler source

The custom RT-FaaS scheduler used by the EDF measurements is available in:

```
rtfaas-scheduler/
```

It contains the Python scheduler package, a Dockerfile, RBAC, and a Kubernetes deployment manifest. Build and deploy it with:

```
cd rtfaas-scheduler
docker build -t botondbencs/rtfaas-scheduler:min-cpu .
docker push botondbencs/rtfaas-scheduler:min-cpu
kubectl apply -f k8s/rbac.yaml
kubectl apply -f k8s/deployment.yaml
kubectl -n kube-system rollout status deploy/rtfaas-scheduler
```

Verify it with:

```
kubectl -n kube-system get pod -l app=rtfaas-scheduler -o wide
kubectl -n kube-system logs -f deploy/rtfaas-scheduler --tail=100
```

## Run the simulator in Kubernetes
1. Build the robot simulator in Docker
   ```
   cd Influxdb
   docker build . -t influxtmp
   cd ../Robot
   bash build.sh
   ```
2. Install InfluxDB
   ```
   cd Influxdb
   bash influxdb_install.sh
   ```
3. Install Grafana
   ```
   cd Grafana
   kubectl apply -f grafana.yaml
   ```
4. Run the robot simulator
   ```
   cd Robot
   kubectl apply -f deployment.yaml
   ```
5. The pid response timeout of the robot can be adjusted in the command section of the container in the deployment.yaml file
6. The robot needs a server that calculates the PID values to start the PID server issue the following commands
   ```
   cd Server
   bash build.sh
   kubectl apply -f deployment.yaml
   kubectl apply -f service.yaml
   ```
## Run the PID server on top of OpenFaaS

1. Install openfaas by using the modified faas-netes module
   ```
   curl -sSLf https://raw.githubusercontent.com/helm/helm/master/scripts/get-helm-3 | bash
   kubectl apply -f https://raw.githubusercontent.com/openfaas/faas-netes/master/namespaces.yml
   helm repo add openfaas https://openfaas.github.io/faas-netes/
   helm repo update  && \
   helm upgrade openfaas --install openfaas/openfaas \
      --namespace openfaas \
      --set functionNamespace=openfaas-fn \
      --set generateBasicAuth=true \
      --set openfaasPRO=false \
      --set faasnetes.image=szefoka/faas-netes:latest \
      --set openfaasImagePullPolicy=Always
   
   PASSWORD=$(kubectl -n openfaas get secret basic-auth -o jsonpath="{.data.basic-auth-password}" | base64 --decode) && \
   echo "OpenFaaS admin password: $PASSWORD"
   ```
2. Build the patched FaaS CLI tool
   ```
   git clone https://github.com/szefoka/faas-cli.git
   cd faas-netes
   git checkout edf
   go build
   cp faas-cli /usr/bin/
   faas-cli login -u admin -p $PASSWORD -g localhost:31112
   ```
3. Deploy the PID server function
   ```
   cd functions
   faas-cli deploy -f pidserver.yaml -g localhost:31112
   ```
   If you want to made modifications on the server, you can do it in the pidserver/handler.py. In this case you have to build and push the function befor deploying the changes. To be able to push make sure that you are logged in with your docker account. (docker loging -u <username>), and change the image tag of the pidserver.yaml accordingly.
   ```
   faas-cli build -f pidserver.yaml
   faas-cli push -f pidserver.yaml   
   faas-cli deploy -f pidserver.yaml -g localhost:31112
   ```
   
   In the template folder you can find the Flask server that runs the PID calculator and the Dockerfile, if you want to make modifications in the installed packages
   
4. To run the PID server with EDF you should define the EDF tag and the runtime, deadline and period values. These values are in milliseconds and they are going to be passed to the function as Env vars.
   ```yaml
   version: 1.0
   provider:
     name: openfaas
     gateway: http://127.0.0.1:8080
   functions:
     pidserver:
       lang: python3-edf
       handler: ./pidserver
       image: szefoka/pidserver:latest
       constraints:
       - "job=pid"
       EDF:
         runtime: "50"
         deadline: "1000"
         period: "1000"
   ```
   If you want to run the PID server without EDF, simply delete the EDF section.
   
   When using EDF the underlying OpenFaaS will calculate and translate the CPU resources used by the function throug EDF.

   When not using EDF you can define the amount of CPU resources you want to assign for the function instance.

   In this case change the pidserver.yaml as shown below, and change the values according to your needs:
   ```yaml
   version: 1.0
   provider:
     name: openfaas
     gateway: http://127.0.0.1:8080
   functions:
     pidserver:
       lang: python3-edf
       handler: ./pidserver
       image: szefoka/pidserver:latest
       constraints:
       - "job=pid"   
       limits:
         cpu: 100m
       requests:
         cpu: 100m
   ```
   In this work we use guaranteed QoS classes, therefore make sure that the values defined under limits and requests are always the same. Another option to do this, define only the limits. 

## Adjusting the robot's response timeout
The completion time of PID calculation does not depend on the input parameters, therefore it uses almost the same amount of CPU ticks in  each invocation.
Almost, because parsing the input and building the output json strings can vary a bit.
To have an idea of the completion time of an invocation we can measure it by using the Linux Perf tool and Hey, that is an HTTP load generator application.
Here you can find 6 example json strings that can be received by the pid server
```
{"Derivator":-0.7239541411399841,"Integrator":-3.0,"Integrator_max":3.0,"Integrator_min":-3.0,"Kd":6.0,"Ki":0.10000000149011612,"Kp":7.0,"current_value":0.08886042982339859,"set_point":0.0}
{"Derivator":1.4746179580688477,"Integrator":0.2876659631729126,"Integrator_max":3.0,"Integrator_min":-3.0,"Kd":6.0,"Ki":0.10000000149011612,"Kp":7.0,"current_value":-1.6177839040756226,"set_point":0.0}
{"Derivator":-2.405780792236328,"Integrator":-3.0,"Integrator_max":3.0,"Integrator_min":-3.0,"Kd":0.009999999776482582,"Ki":0.004999999888241291,"Kp":0.009999999776482582,"current_value":2.1491098403930664,"set_point":0.0}
{"Derivator":-0.42415735125541687,"Integrator":-3.0,"Integrator_max":3.0,"Integrator_min":-3.0,"Kd":0.009999999776482582,"Ki":0.004999999888241291,"Kp":0.009999999776482582,"current_value":-0.16618366539478302,"set_point":0.0}
{"Derivator":1.042830228805542,"Integrator":3.0,"Integrator_max":3.0,"Integrator_min":-3.0,"Kd":0.009999999776482582,"Ki":0.004999999888241291,"Kp":0.009999999776482582,"current_value":0.05838686227798462,"set_point":0.0}
{"Derivator":-0.6928533315658569,"Integrator":2.211747169494629,"Integrator_max":3.0,"Integrator_min":-3.0,"Kd":0.009999999776482582,"Ki":0.004999999888241291,"Kp":0.009999999776482582,"current_value":2.1661009788513184,"set_point":0.0}
```
Passing one of these to hey we can fake a robot calling the PID server by the following command
```
./hey_linux_amd64 -c 1 -n 10 -q 1 -m POST \
-d {"Derivator":-0.7239541411399841,"Integrator":-3.0,"Integrator_max":3.0,"Integrator_min":-3.0,"Kd":6.0,"Ki":0.10000000149011612,"Kp":7.0,"current_value":0.08886042982339859,"set_point":0.0} \
-o csv http://10.96.52.184:5000/pid
```
Command explanation:
1. -c 1 - concurrency is 1, let's not overcomplicate our job
2. -n 10 - 10 measurements
3. -q 1 - 1 request / sec
4. -m POST - Using HTTP POST method
5. -d .... - request body
6. -o csv - csv output for the sake of legibility
7. address of the server, this can be acquired by issuing
   ```
   kubectl get services
   ```
So we should open two terminal windows. In one we will be signed in to our Kubernetes master node, in the other we will be signed in to our Kubernetes worker node.
On the worker node we issue this huge command:
   ```
   perf record -a --per-thread  -e sched:sched_switch  -e sched:sched_stat_sleep -e sched:sched_stat_wait -e sched:sched_process_wait -e sched:sched_stat_blocked -e sched:sched_process_exec -e sched:sched_process_fork -e sched:sched_process_exit -e sched:sched_stat_runtime -e sched:sched_wakeup -e sched:sched_wakeup_new -e sched:sched_waking -eworkqueue:workqueue_{queue_work,activate_work,execute_start,execute_end}
   ```
While on the master node we issue the Hey command from above. But I know that you are lazy to scroll back, so here it is:
   ```
   ./hey_linux_amd64 -c 1 -n 10 -q 1 -m POST -d {"Derivator":-0.7239541411399841,"Integrator":-3.0,"Integrator_max":3.0,"Integrator_min":-3.0,"Kd":6.0,"Ki":0.10000000149011612,"Kp":7.0,"current_value":0.08886042982339859,"set_point":0.0} -o csv http://10.96.52.184:5000/pid
   ```
After Hey has returned, we switch to the worker node and press Ctrl+C to stop the perf record command
To see what we have recoreded the following is needed
   ```
   perf sched timehist| grep python
   ```
You will see something like this:
   ```
           time    cpu  task name                       wait time  sch delay   run time
                        [tid/pid]                          (msec)     (msec)     (msec)
--------------- ------  ------------------------------  ---------  ---------  ---------
  173873.031480 [0006]  python3[253155]                   293.044      0.001      1.181 
  173873.037522 [0007]  python3[365985]                     0.000      0.002      6.360 
  173873.038516 [0007]  python3[365985]                     0.126      0.002      0.868 
  173873.531724 [0006]  python3[253155]                   500.205      0.002      0.038 
  173874.029989 [0006]  python3[253155]                   497.491      0.001      0.773 
  173874.034794 [0007]  python3[365986]                     0.000      0.002      5.084 
  173874.035654 [0007]  python3[365986]                     0.047      0.002      0.812 
  173874.530536 [0006]  python3[253155]                   500.488      0.001      0.059 
  173875.030314 [0006]  python3[253155]                   498.780      0.001      0.997 
  173875.036138 [0000]  python3[365987]                     0.000      0.002      6.116 
  173875.037016 [0000]  python3[365987]                     0.011      0.002      0.865 
  173875.530859 [0006]  python3[253155]                   500.492      0.001      0.051 
  173876.030329 [0006]  python3[253155]                   498.698      0.001      0.771 
  173876.035594 [0007]  python3[365989]                     0.000      0.002      5.540 
  173876.036469 [0007]  python3[365989]                     0.026      0.002      0.848 
  173876.530878 [0006]  python3[253155]                   500.490      0.001      0.057 
  173877.030069 [0006]  python3[253155]                   498.320      0.001      0.871 
  173877.035325 [0007]  python3[365990]                     0.000      0.002      5.544 
  173877.036210 [0007]  python3[365990]                     0.043      0.002      0.841 
  173877.530627 [0006]  python3[253155]                   500.489      0.001      0.068 
  173878.030325 [0006]  python3[253155]                   498.641      0.001      1.056 
  173878.036157 [0007]  python3[365991]                     0.000      0.002      6.126 
  173878.037022 [0007]  python3[365991]                     0.006      0.002      0.858 
  173878.530861 [0006]  python3[253155]                   500.488      0.001      0.047 
  173879.030356 [0006]  python3[253155]                   498.603      0.001      0.891 
  173879.035839 [0007]  python3[365992]                     0.000      0.002      5.765 
  173879.036738 [0007]  python3[365992]                     0.047      0.002      0.851 
  173879.530898 [0006]  python3[253155]                   500.486      0.001      0.054 
  173880.030068 [0006]  python3[253155]                   498.324      0.001      0.845 
  173880.035294 [0007]  python3[365993]                     0.000      0.002      5.514 
  173880.036176 [0007]  python3[365993]                     0.035      0.002      0.845 
  173880.530612 [0006]  python3[253155]                   500.491      0.003      0.053 
  173881.029762 [0006]  python3[253155]                   498.437      0.001      0.712 
  173881.034333 [0007]  python3[365994]                     0.000      0.002      4.812 
  173881.035162 [0007]  python3[365994]                     0.040      0.002      0.789 
  173881.530304 [0006]  python3[253155]                   500.488      0.001      0.053 
  173882.029748 [0006]  python3[253155]                   498.693      0.001      0.751 
  173882.034607 [0007]  python3[365995]                     0.000      0.002      5.102 
  173882.035451 [0007]  python3[365995]                     0.020      0.002      0.824 
  173882.530306 [0006]  python3[253155]                   500.491      0.001      0.066 
  173883.030820 [0006]  python3[253155]                   500.490      0.003      0.023 
  173883.531326 [0006]  python3[253155]                   500.488      0.001      0.017 
  173884.031469 [0006]  python3[253155]                   500.123      0.001      0.019 
  173884.532018 [0006]  python3[253155]                   500.496      0.001      0.052 
   ```
Let me explain, what is that thingy up there. We can see the behaviour of the PID server under the load that we have generated. The PID server is a multi-threaded python flask server, that has a main thread, if you watch it carefully you can see that PID (now process id) 253155 is returning. This is the main thread of the PID server. The rest of the PIDs are python processes that are started by the main thread and responsible for serving the invocation requests. 
Let's focus on the following lines:
   ```
  173873.031480 [0006]  python3[253155]                   293.044      0.001      1.181 
  173873.037522 [0007]  python3[365985]                     0.000      0.002      6.360 
  173873.038516 [0007]  python3[365985]                     0.126      0.002      0.868 
  173873.531724 [0006]  python3[253155]                   500.205      0.002      0.038 
  173874.029989 [0006]  python3[253155]                   497.491      0.001      0.773 
  173874.034794 [0007]  python3[365986]                     0.000      0.002      5.084 
  173874.035654 [0007]  python3[365986]                     0.047      0.002      0.812 
   ```
We can see that the main thread 253155 started 365985 that was running for 6.360 + 0.868ms, then it has sent the PID response to the robot.

Then 253155 received another request and started 365986 that was running for 5.084 + 0.812ms to calculate the return value.

The first column shows the time when the given event has happened.

The first line indicates that the invocation request arrived at around 173873.031480 (sec.usec) Then the second invocation arrived at 173874.029989 just a second later.

If you examine the output a bit more, you can see that the time needed to calculate the PID value is around 6-7ms.

Now, that we know the time the software needs we can adjust the response timeout of our robot. We know that it calls the PID server 3 times, that means something between 18-21 ms. However we need to consider the network delay, so let's add a couple of milliseconds more. 

From here it should be easy to configure the robot....

We can also adjust the CPU consumption of the PID server. We know that the scheduling window of the Linux CFS scheduler is set to 100ms by default on Ubuntu. So in this case we have N milliseconds in a 100ms long scheduling window. If we have a multi-core CPU we can add more than 100ms of runtime in the 100ms scheduling window.

We know that the PID server runs for 6-7 milliseconds to be able to serve a single request. We also know that the robot invokes it 3 times in one "balancing" cycle, therefore if we have a single robot that invokes the PID server 3 times every 100ms (Robot/main.cpp:215), we need to set the PID server's minimum CPU consumption to somewhere between 18-21ms in a 100ms long scheduling window, which translates to 21% of CPU usage. However, if we scale out the robot, we need to increase the PID server's CPU allocation accordingly. This means 36-42% in case of 2 and 54-63% in case of 3 robots, etc.

## Robot with graphical output - Only for local testing

To compile the robot issue the following two commands
``` 
g++ -std=c++14 -Wall -c main_graphic.cpp  -g -pthread -lrt -g -lGL -lGLU -lglut  -lGLEW -lcurl
g++ -std=c++14 -Wall -o main_graphic main_graphic.o -g -pthread -lrt -g -lGL -lGLU -lglut -lGLEW -lcurl
```

### Run tests with the graphical robot

To run the testbench open two terminal windows. In one run the compiled c++ application
```
./main_graphic
```
In the other terminal window run pidserver.py by issuing
```
python3 pidserver.py
```
