run the following command in terminal to merge model

&& python FileSplitter.py -i frozen_inference_graph.pb -n 5 -s

to split,  run the command

&& python FileSplitter.py -i frozen_inference_graph.pb -n 5 -j
