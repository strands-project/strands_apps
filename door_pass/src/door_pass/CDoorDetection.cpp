#include "door_pass/CDoorDetection.h"

SDoor detectDoor(std::vector<float> d,float angle_min,float angle_increment,size_t num_ranges,float maxDistance)
{

	int rayThreshold = 20;
	float edgeThreshold = 0.5;		//min range step to detect an edge 
	float minDoorWidth = 0.65;		//minimal doorWidth 
	float maxDoorWidth = 0.85;		//maximal doorWidth
	bool debug = true;
	float auxiliaryDistance = 0.3;		//used to make the door position more precise
	float doorApproachDistance = 0.5;	//the distance from the door before appempting to pass through 

	SDoor result;
	result.found = false;
	float x[num_ranges];
	float y[num_ranges];
	float dx,dy,da;
	float angle;
	int leftArray[num_ranges],rightArray[num_ranges];
	int leftNum,rightNum;
	float doorAngle,doorCentreX,doorCentreY,auxX,auxY,doorWidth;
	int left,right,leftIndex,rightIndex;
	bool found = false;
	for (int i = 0; i <= num_ranges; i++)
	{
		angle = angle_min+i*angle_increment;
		d[i] = fmin(d[i],maxDistance);
		x[i] = d[i]*cos(angle)+0.07;
		y[i] = d[i]*sin(angle);
	}
	/*calculate candidates*/
	leftNum = rightNum = 0;
	for (int i = 1; i <= num_ranges-1; i++) if ((d[i]-d[i-1])<-edgeThreshold) leftArray[leftNum++] = i;
	for (int i = 1; i <= num_ranges-1; i++) if ((d[i]-d[i-1])>+edgeThreshold) rightArray[rightNum++] = i-1;
	if (debug) fprintf(stdout,"Straightforward candidates left: %i right: %i\n",leftNum,rightNum);
	if (leftNum*rightNum > 0){
		for (int i=0;i<leftNum && found == false;i++){
			for (int j=0;j<rightNum && found == false;j++){
				left = leftArray[i];
				right = rightArray[j];
				doorWidth = 0;
				if (left-right > rayThreshold){
					doorWidth = sqrt((x[right]-x[left])*(x[right]-x[left])+(y[right]-y[left])*(y[right]-y[left]));
					if (doorWidth > minDoorWidth && doorWidth < maxDoorWidth) found = true;
				}
				if (debug && found) fprintf(stdout,"Possible doors: %i %i %f \n",left,right,doorWidth);
			}	
		}
	}

	/*no straighforward candidates*/
	float minDist = 1000;
	int index=0;
	float dist=0;
	if (found == false && leftNum+rightNum >0){
		for (int i = 0;i<leftNum && found==false;i++){
			left = leftArray[i];
			minDist = 1000;
			right = 0;
			for (int j=2;j<left;j++){
				dx = (x[j]-x[left]);
				dy = (y[j]-y[left]);
				dist = sqrt(dx*dx+dy*dy);
				if (dist<minDist){
					minDist = dist;
					right = j;
				}
			}
			if (debug) fprintf(stdout,"Left estimate %i %i %f\n",left,right,minDist);
			doorAngle = M_PI/2+atan2(y[left]-y[right],x[left]-x[right]);
			doorCentreX = (x[right]+x[left])/2;	
			doorCentreY = (y[right]+y[left])/2;
			auxX = doorCentreX+cos(doorAngle)*auxiliaryDistance;
			auxY = doorCentreY+sin(doorAngle)*auxiliaryDistance;
			if (debug) printf("Door position: %f %f %f %f %f \n",doorCentreX,doorCentreY,doorAngle,auxX,auxY);
			minDist = 100;
			for (int i = 0;i<right;i++){
				dx = (x[i]-auxX);
				dy = (y[i]-auxY);
				if (sqrt(dx*dx+dy*dy)<minDist){
					minDist = sqrt(dx*dx+dy*dy);
					index = i;
				}
			}
			minDist = 100;
			right = index;
			for (int i = left;i<num_ranges;i++)
			{
				dx = (x[i]-auxX);
				dy = (y[i]-auxY);
				if (sqrt(dx*dx+dy*dy)<minDist){
					minDist = sqrt(dx*dx+dy*dy);
					index = i;
				}
			}
			left = index;
			doorWidth = sqrt((x[right]-x[left])*(x[right]-x[left])+(y[right]-y[left])*(y[right]-y[left]));
			if (debug) fprintf(stdout,"Additional %i %i %f \n",left,right,doorWidth);
			if (doorWidth > minDoorWidth && doorWidth < maxDoorWidth&& left-right > rayThreshold) found = true;
		}

		for (int i = 0;i<rightNum && found==false;i++){
			right = rightArray[i];
			minDist = 1000;
			left = right +1;
			for (int j=right+2;j<num_ranges-1;j++){
				dx = (x[j]-x[right]);
				dy = (y[j]-y[right]);
				dist = sqrt(dx*dx+dy*dy);
				if (dist<minDist){
					minDist = dist;
					left = j;
				}
			}

			if (debug) fprintf(stdout,"Estimate %i %i \n",left,right);
			doorAngle = M_PI/2+atan2(y[left]-y[right],x[left]-x[right]);
			doorCentreX = (x[right]+x[left])/2;	
			doorCentreY = (y[right]+y[left])/2;
			auxX = doorCentreX+cos(doorAngle)*auxiliaryDistance;
			auxY = doorCentreY+sin(doorAngle)*auxiliaryDistance;
			minDist = 100;
			if (debug) printf("Door position: %f %f %f %f %f \n",doorCentreX,doorCentreY,doorAngle,auxX,auxY);

			for (int i = 0;i<left;i++){
				dx = (x[i]-auxX);
				dy = (y[i]-auxY);
				if (sqrt(dx*dx+dy*dy)<minDist){
					minDist = sqrt(dx*dx+dy*dy);
					right = i;
				}
			}

			minDist = 1000;
			for (int i = right+2;i<num_ranges;i++){
				dx = (x[i]-auxX);
				dy = (y[i]-auxY);
				if (sqrt(dx*dx+dy*dy)<minDist){
					minDist = sqrt(dx*dx+dy*dy);
					left = i;
				}
			}
			doorWidth = sqrt((x[right]-x[left])*(x[right]-x[left])+(y[right]-y[left])*(y[right]-y[left]));
			if (doorWidth > minDoorWidth && doorWidth < maxDoorWidth && left-right > rayThreshold) found = true;
			if (debug) fprintf(stdout,"Additional %i %i %f %i\n",left,right,doorWidth,found);
		}
	}
	if (found){
		result.found = true;
		result.doorCentreX = (x[right]+x[left])/2;	
		result.doorCentreY = (y[right]+y[left])/2;
		result.doorWidth = sqrt((x[right]-x[left])*(x[right]-x[left])+(y[right]-y[left])*(y[right]-y[left]));
		if (debug) printf("Corrected: %i %i %f \n",right,left,doorWidth);
		result.doorAngle = M_PI/2+atan2(y[left]-y[right],x[left]-x[right]);	
		if (debug) printf("Detected door width %f %f %f %f \n",result.doorWidth,result.doorCentreX,result.doorCentreY,result.doorAngle);
		result.auxX = result.doorCentreX + cos(result.doorAngle)*doorApproachDistance;
		result.auxY = result.doorCentreY + sin(result.doorAngle)*doorApproachDistance;
	}
	return result;
}
