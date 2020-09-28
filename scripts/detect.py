# %%
import torch
import torchvision.models.detection
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from PIL import Image
import time
import sys

model_file = "/home/nvidia/deconbot/src/turtlebot3/turtlebot3_bringup/scripts/model_test2.pth"
threshold = 0.7

def prepare_model():

	model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
	
	num_classes = 4
	in_features = model.roi_heads.box_predictor.cls_score.in_features
	model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
	
	
	device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
	tr = torchvision.transforms.ToTensor()
	
	model.to(device)
	checkpoint = torch.load(model_file, map_location='cpu')
	model.load_state_dict(checkpoint['model'])
	model.eval()

	return model, tr

def detect_image(imagePath, model, tr):
	
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
	image = tr(Image.open(imagePath).convert("RGB"))
	image = [image.to(device)]

	model_time = time.time()
	outputs = model(image)
	outputs = [{k: v.to(torch.device('cpu')) for k, v in t.items()} for t in outputs]
	model_time = time.time() - model_time
	
	out_images = []
	for img, output in zip(image, outputs):
	    pred_boxes = []
	    pred_labels = []
	    cpu_image = img.cpu()
	    output_boxes = output["boxes"].cpu()
	    labels = output["labels"].cpu()
	    scores = output["scores"].cpu()
	    for box, score, label in zip(output_boxes.tolist(), scores.tolist(), labels.tolist()):
	        if score > threshold:
	            pred_boxes.append([(box[0]+box[2])/2, (box[1]+box[3])/2, box[2]-box[0], box[3]-box[1]])
	            pred_labels.append(label)
	
	del model
	del image
	del device
	torch.cuda.empty_cache()
	return pred_boxes, pred_labels
