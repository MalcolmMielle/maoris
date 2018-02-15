#include "Evaluation.hpp"

void AASS::maoris::Evaluation::compare(const cv::Mat& seg, const cv::Mat& GT_segmentation, double time, const std::string& file)
{
	
// 	cv::Mat img_hist_equalizedgt;
// 	cv::equalizeHist(GT_segmentation, img_hist_equalizedgt);
// 	cv::imshow("GT", img_hist_equalizedgt);
// 	
// 	cv::Mat img_hist_equalized;
// 	cv::equalizeHist(seg, img_hist_equalized);
// 	cv::imshow("seg", img_hist_equalized);
// 	
// 	
// 	cv::waitKey(0);
	
	double min, max;
	cv::minMaxLoc(GT_segmentation,&min,&max);
	
	float rows = GT_segmentation.rows;
	float cols = GT_segmentation.cols;
	float proper_size = rows*cols/1000;
	proper_size = proper_size/1000;
	
	
	addTime(time);
	boost::filesystem::path p(file);
	std::string name = p.filename().stem().string();
	addName(name);
	_proper_size.push_back(proper_size);
	_labels.push_back(max);
	
	
	
	assert(seg.rows == GT_segmentation.rows);
	assert(seg.cols == GT_segmentation.cols);
				
	
	
	/******** THIS************/
	
//	AASS::maoris::results Regions;
//	AASS::maoris::compare_images(GT_segmentation, seg, Regions);
// 	addPrecision(Regions.precision);
// 	addRecall(Regions.recall);
// 	addInverseRecall(Regions.inverse_recall);

	/******** OR THIS************/
	
	compareImagesUnbiased(GT_segmentation, seg);
	computeMeasures();
	
	/********************/
	
	std::cout << "Compared" << std::endl;
	
// 	std::cout << "pixel prec " << pixel_precision << std::endl;				
	std::cout << "Go for extract" << std::endl;
	
// 				extract_results(Regions, Precisions, Recalls, inverse_recall);
	
	
//	std::cout << " OldFurniture Precision: " <<Regions.precision << " Recall: "<< Regions.recall << " Inverse recall " << Regions.inverse_recall << std::endl;
	
	std::cout << " No_Furniture Precision: " << _precision[_precision.size() - 1] << " Recall: "<< _recall[_precision.size() - 1] << " Inverse recall " << _inverse_recall[_precision.size() - 1] << " time: "<< _time[_precision.size() - 1] <<" Labels " << max <<"  size " << proper_size << std::endl;
	
	
}

void AASS::maoris::Evaluation::exportAll(const std::string& file_out)
{
				
	calculate();
	
	std::string result_file = file_out;
	std::ofstream myfile;
	if(!exists_test3(result_file)){
		myfile.open (result_file);
		myfile << "# precision recall inverse_recall time labels size fscore gscore dorscore matthewsCC accuracy tp tn fp fn\n";
	}
	else{
		myfile.open (result_file, std::ios::out | std::ios::app);
	}
	
	if (myfile.is_open())
	{
		for(int i = 0 ; i < _precision.size() ; ++i){
			
			myfile << _name[i] << " " << _precision[i] << " " << _recall[i] << " " << _inverse_recall[i] << " " << _time[i] << " " << _labels[i] << " " << _proper_size[i] << " " << _f1_score_individual[i] << " " << _g_score_individual[i] << " " << _dor_individual[i] << " " << _matthewCC_individual[i] << " " << _accuracy_individual[i] <<  " " << _tp[i] << " " << _tn[i] << " " << _fp[i] << " " << _fn[i] << "\n";
			
		}
		
		myfile << "\n\n# precision_mean recall_mean inverse_recall_mean precision_sd recall_sd inverse_recall_sd time_mean f1_score g_score dor_score matthewCC sd_mCC matthiew_median accuracy\n";
		myfile << _mean_p << " " << _mean_r << " " << _mean_ir << " " << _sd_p << " " << _sd_r << " " << _sd_ir << " " << mean<double>(_time) << " " <<_f1score << " " << _gscore << " " << _dor << " " << _matthewCC << " " << _sd_mCC << " " << _matthewCC_median << " " << _accuracy << "\n";
		myfile.close();
		
		
	}
	else std::cout << "Unable to open file";
	
}

void AASS::maoris::Evaluation::computeMeasures()
{
	int i;
	if(_matthewCC_individual.size() > 0){
		i = _matthewCC_individual.size();
	}
	else{
		i = 0;
	}
	for(i ; i < _precision.size() ; ++i){
		_f1_score_individual.push_back(fscore<double>(_precision[i], _recall[i]));
		_g_score_individual.push_back(gscore<double>(_precision[i], _recall[i]));
		_matthewCC_individual.push_back(matthewCC<double>(_tp[i], _fp[i], _tn[i], _fn[i]));
		_dor_individual.push_back(DOR<double>(_tp[i], _fp[i], _tn[i], _fn[i]));
		_accuracy_individual.push_back(accuracy<double>(_tp[i], _fp[i], _tn[i], _fn[i]));
	}

}



void AASS::maoris::Evaluation::calculate()
{
	_mean_p = mean<double>(_precision);
	
	assert(!std::isnan(_mean_p));
	_mean_r = mean<double>(_recall);
	_mean_ir = mean<double>(_inverse_recall);
	
	_sd_p = sd<double>(variance<double>(_precision, _mean_p));
	_sd_r = sd<double>(variance<double>(_recall, _mean_r));
	_sd_ir = sd<double>(variance<double>(_inverse_recall, _mean_ir));
	
	//Get all tp fn...
	double fn = 0, fp = 0, tn = 0, tp = 0;
	for(int i = 0 ; i < _tn.size() ; ++i){
		fn = fn + _fn[i];
		tn = tn + _tn[i];
		tp = tp + _tp[i];
		fp = fp + _fp[i];
	}
	
	_f1score = fscore<double>(_mean_p, _mean_r);
	_gscore = gscore<double>(_mean_p, _mean_r);
	_matthewCC = mean<double>(_matthewCC_individual);
	_matthewCC_median = median<double>(_matthewCC_individual);
	_sd_mCC = sd<double>(variance<double>(_matthewCC_individual, _matthewCC));
	_dor = mean<double>(_dor_individual);
	_accuracy = mean<double>(_accuracy_individual);

	auto minmax = std::minmax_element(_matthewCC_individual.begin(), _matthewCC_individual.end());
	_max_mcc = *(minmax.second);
	_min_mcc = *(minmax.first);

    assert(_max_mcc >= 0);
    assert(_max_mcc <= 1);
    assert(_min_mcc >= 0);
    assert(_min_mcc <= 1);


//	_min_mcc = std::min_element(_matthewCC_individual);

}


void AASS::maoris::Evaluation::compareImagesUnbiased(cv::Mat GT_segmentation_in, cv::Mat DuDe_segmentation_in)
{
	
// 	cv::imshow("GT", GT_segmentation_in);
// 	cv::imshow("Seg", DuDe_segmentation_in);
// 	cv::waitKey(0);
// 	
	std::vector<double> precisions;
	std::vector<double> recalls;
	std::vector<double> inverse_recalls;
	                  
	std::map<int,int> segmented2GT_tags;
			
	cv::Mat GT_segmentation   = cv::Mat::zeros(GT_segmentation_in.size(),CV_8UC1);
	cv::Mat DuDe_segmentation = cv::Mat::zeros(GT_segmentation_in.size(),CV_8UC1);
	
	
	GT_segmentation_in  .convertTo(GT_segmentation, CV_8UC1);
	DuDe_segmentation_in.convertTo(DuDe_segmentation, CV_8UC1);			
	tag2tagMapper gt_tag2mapper,DuDe_tag2mapper;
	
	std::map<int, std::vector<cv::Point> > dude_points;
	std::map<int, std::vector<cv::Point> > gt_points;
		
	int nb_of_pixels = 0;
	
	for(int x=0; x < GT_segmentation.size().width; x++){
		for(int y=0; y < GT_segmentation.size().height; y++){
			cv::Point current_pixel(x,y);
								
			int tag_GT   = GT_segmentation.at<uchar>(current_pixel);
			int tag_DuDe  = DuDe_segmentation.at<uchar>(current_pixel);
			
// 			std::cout << "Tags gt   " << tag_GT << std::endl;
// 			std::cout << "Tags dude " << tag_DuDe << std::endl;
			
			if(tag_DuDe>0 && tag_GT>0 ){
// 				std::cout << "Add" << std::endl;
				gt_tag2mapper  [tag_GT][tag_DuDe].push_back(current_pixel);
				DuDe_tag2mapper[tag_DuDe][tag_GT].push_back(current_pixel);
				
				dude_points[tag_DuDe].push_back(current_pixel);
				gt_points[tag_GT].push_back(current_pixel);
				
				++nb_of_pixels;
			}
		}
	}
	
	
	AllZoneAsso allAsso;
	allAsso.FromTag(DuDe_tag2mapper);
//    std::cout << "There is " << allAsso.size() << " asso " << std::endl;


    allAsso.sort();
//    std::cout << "There is " << allAsso.size() << " asso " << std::endl;

    allAsso.calculateAsso();
	
 	std::cout << "There is " << allAsso.size() << " asso " << std::endl;
 	assert(allAsso.size() != 0);
	
	
	std::vector<double> tp;
	std::vector<double> tn;
	std::vector<double> fp;
	std::vector<double> fn;

// 	double tp = 0 ;
// 	double tn = 0 ;
// 	double fp = 0 ;
// 	double fn = 0 ;
	
	for (auto &association : allAsso.associations) {
// 		std::cout << "FIRST ZONE" << std::endl;
		int seg = association.first;
		int gt = association.second;
		
		unsigned long max_intersection = DuDe_tag2mapper[seg][gt].size();
        unsigned long total_points = dude_points[seg].size();
		unsigned long total_gt_point = gt_points[gt].size();
		double tp_t = max_intersection;
		double fp_t = total_points - max_intersection;
		double tn_t = nb_of_pixels - (fp_t - total_gt_point);
		double fn_t = total_gt_point - tp_t;
		
		tp.push_back(tp_t);
		fp.push_back(fp_t);
		tn.push_back(tn_t);
		fn.push_back(fn_t);
		
 		std::cout << "tp fp tn fn " << tp_t << " " << fp_t << " " << tn_t << " " << fn_t << std::endl;
		
		assert(!std::isnan(tp_t + fp_t));
		assert(!std::isnan(tp_t + fn_t));
		assert(!std::isnan(fp_t + tn_t));
		
		precisions.push_back(tp_t / (tp_t + fp_t));
		recalls.push_back(tp_t / (tp_t + fn_t));
		inverse_recalls.push_back(fp_t / (fp_t + tn_t));
		
		assert(!std::isnan(tp_t / (tp_t + fp_t)));
		assert(!std::isnan(tp_t / (tp_t + fn_t)));
		assert(!std::isnan(fp_t / (fp_t + tn_t)));
		
		
// 		cv::Mat DuDe_segmentation_draw = cv::Mat::zeros(GT_segmentation.size(),CV_8UC1);
// 		for(auto it3 = dude_points[seg].begin() ;  it3 != dude_points[seg].end() ; ++it3){
// 			DuDe_segmentation_draw.at<uchar>(*it3) = 255;
// 		}
// 		cv::imshow("dude", DuDe_segmentation_draw);
// 		
// 		cv::Mat GT_segmentation_draw = cv::Mat::zeros(GT_segmentation.size(),CV_8UC1);
// 		for(auto it3 = gt_points[gt].begin() ;  it3 != gt_points[gt].end() ; ++it3){
// 			GT_segmentation_draw.at<uchar>(*it3) = 255;
// 		}
// 		cv::imshow("GT temp", GT_segmentation_draw);
// 		cv::waitKey(0);
		
		
		
		
	}
	
	
	
	
	
	
// 	//calculate true positive and all only for the segmentation given by the user
// 	for( tag2tagMapper::iterator it = DuDe_tag2mapper.begin(); it!= DuDe_tag2mapper.end(); it++ ){
// 		
// 		
// //				std::cout << "   " << it->first << " connected to "<< std::endl;
// 		
// 		tag2points inside = it->second;
// 		auto gt_equivalent_seg = it->second.begin();
// 		int max_intersection=0; 
// 		for( tag2points::iterator it2 = inside.begin(); it2!= inside.end(); it2++ ){
// 			if (it2->second.size() > max_intersection){
// 				max_intersection = it2->second.size();
// 				gt_equivalent_seg = it2;
// 			}	
// 		}
// 		
// 		
// 
// 		
// 		int total_points = dude_points[it->first].size();
// 		int total_gt_point = gt_points[gt_equivalent_seg->first].size();
// 		double tp_t = max_intersection;
// 		double fp_t = total_points - max_intersection;
// // 		double tn_t = nb_of_pixels - (fp_t - total_gt_point);
// // 		double fn_t = total_gt_point - tp_t;
// 		
// 		tp =  tp + tp_t;
// 		fp = fp + fp_t;
// 		
// // 		precisions.push_back(tp_t / (tp_t + fp_t));
// // 		recalls.push_back(tp_t / (tp_t + fn_t));
// // 		inverse_recalls.push_back(fp_t / (fp_t + tn_t));
// 		
// // 		std::cout << "p : " << tp_t / (tp_t + fp_t) << " r : " << tp_t / (tp_t + fn_t) << " ir : " << fp_t / (fp_t + tn_t) << std::endl;
// 		
// // 		cv::Mat DuDe_segmentation_draw = cv::Mat::zeros(GT_segmentation.size(),CV_8UC1);
// // 		for(auto it3 = dude_points[it->first].begin() ;  it3 != dude_points[it->first].end() ; ++it3){
// // 			DuDe_segmentation_draw.at<uchar>(*it3) = 255;
// // 		}
// // 		cv::imshow("dude", DuDe_segmentation_draw);
// // 		
// // 		cv::Mat GT_segmentation_draw = cv::Mat::zeros(GT_segmentation.size(),CV_8UC1);
// // 		for(auto it3 = gt_points[gt_equivalent_seg->first].begin() ;  it3 != gt_points[gt_equivalent_seg->first].end() ; ++it3){
// // 			GT_segmentation_draw.at<uchar>(*it3) = 255;
// // 		}
// // 		cv::imshow("GT temp", GT_segmentation_draw);
// // 		cv::waitKey(0);
// 		
// 	}
	
// 	for(auto it = precisions.begin() ; it != precisions.end() ; ++it){
// 		std::cout << "p : " << *it << std::endl;
// 	}
// 	exit(0);

	//All point should be segmented thus no tn;
// 	tn = 0;
	
// 	assert(nb_of_pixels >= tp);
// 	assert(nb_of_pixels >= fp);
	
// 	fn = nb_of_pixels - tp - fp;
	
// 	assert(fn == 0);
	
// 	assert(nb_of_pixels >= fn);
	
	
	
	_precision.push_back(mean<double>(precisions));
	_recall.push_back(mean<double>(recalls));
	_inverse_recall.push_back(mean<double>(inverse_recalls));
	_tp.push_back(mean<double>(tp));
	_fp.push_back(mean<double>(fp));
	_tn.push_back(mean<double>(tn));
	_fn.push_back(mean<double>(fn));

    std::cout << "Mean precision " << mean<double>(precisions) << std::endl;
	assert(std::isnan(mean<double>(precisions)) == false);
	
	
}



/********************************************/






void AASS::maoris::compare_images(cv::Mat GT_segmentation_in, cv::Mat DuDe_segmentation_in, AASS::maoris::results& res_out)
{
					
	std::vector<float> Precisions;
	std::vector<float> Recalls;
	std::vector<float> inverse_recalls;
	
	std::map<int,int> segmented2GT_tags;
			
	cv::Mat GT_segmentation   = cv::Mat::zeros(GT_segmentation_in.size(),CV_8UC1);
	cv::Mat DuDe_segmentation = cv::Mat::zeros(GT_segmentation_in.size(),CV_8UC1);
	
	
	GT_segmentation_in  .convertTo(GT_segmentation, CV_8UC1);
	DuDe_segmentation_in.convertTo(DuDe_segmentation, CV_8UC1);			
	tag2tagMapper gt_tag2mapper,DuDe_tag2mapper;
	
	std::map<int, std::vector<cv::Point> > dude_points;
	std::map<int, std::vector<cv::Point> > gt_points;
	
	
// 	cv::imshow("GTTT raw", GT_segmentation);
// 	cv::Mat img_hist_equalizedgt;
// 	cv::equalizeHist(GT_segmentation, img_hist_equalizedgt);
// 	cv::imshow("GT equal", img_hist_equalizedgt);
// 	cv::imshow("DUDEEE raw", DuDe_segmentation);
// 	cv::waitKey(0);
// 	
	
	int nb_of_pixels = 0;
	
	for(int x=0; x < GT_segmentation.size().width; x++){
		for(int y=0; y < GT_segmentation.size().height; y++){
			cv::Point current_pixel(x,y);
								
			int tag_GT   = GT_segmentation.at<uchar>(current_pixel);
			int tag_DuDe  = DuDe_segmentation.at<uchar>(current_pixel);
			
// 			std::cout << "Tags gt   " << tag_GT << std::endl;
// 			std::cout << "Tags dude " << tag_DuDe << std::endl;
			
			if(tag_DuDe>0 && tag_GT>0 ){
// 				std::cout << "Add" << std::endl;
				gt_tag2mapper  [tag_GT][tag_DuDe].push_back(current_pixel);
				DuDe_tag2mapper[tag_DuDe][tag_GT].push_back(current_pixel);
				
				dude_points[tag_DuDe].push_back(current_pixel);
				gt_points[tag_GT].push_back(current_pixel);
				
				++nb_of_pixels;
			}
		}
	}
	
	std::vector<float> recalls_inside;			
	double cum_precision=0, cum_total=0, cum_recall=0, cum_inverse_recall=0;

	std::cout << "Regions in GT or recall: "<< std::endl;
			
	double tp = 0;
	double fn = 0;
	
	for( tag2tagMapper::iterator it = gt_tag2mapper.begin(); it!= gt_tag2mapper.end(); it++ ){
		cv::Mat DuDe_segmentation_draw = cv::Mat::zeros(GT_segmentation.size(),CV_8UC1);
		
// 		std::cout << "Tag gt " << it->first << std::endl;
//				std::cout << "   " << it->first << " connected to "<< std::endl;
		tag2points inside = it->second;
		int max_intersection=0; 
				
		int gt_tag_max = -1;
// 		std::cout << "Tag seg ";
		for( tag2points::iterator it2 = inside.begin(); it2!= inside.end(); it2++ ){
// 			std::cout << it2->first << " ";
			for(auto it3 = it2->second.begin() ;  it3 != it2->second.end() ; ++it3){
				DuDe_segmentation_draw.at<uchar>(*it3) = 255;
			}
// 			cv::imshow("mat", DuDe_segmentation_draw);
// 			cv::waitKey(0);
// 			total_points += it2->second.size();
			if (it2->second.size() > max_intersection){
				max_intersection = it2->second.size();
				gt_tag_max = it2->first;
			}
//					std::cout << "      " << it2->first << " with "<< it2->second.size() <<" points" << std::endl;					
		}
		
		tp = tp + max_intersection;
// 		std::cout << std::endl;
		
// 		cv::imshow("mat final", DuDe_segmentation_draw);
// 		cv::waitKey(0);		
		int total_points = gt_points[it->first].size();
		fn = fn + total_points - tp;
		
		std::cout << "SIZE " << total_points << " it first " << it->first << std::endl;
		
		
		segmented2GT_tags[gt_tag_max] = it->first;
				std::cout << "   max is " << max_intersection << " that represents " << 100*max_intersection/total_points   << std::endl;
		Recalls.push_back(100*max_intersection/total_points);
		cum_recall += max_intersection;
		cum_total += total_points;
	}	
// 	pixel_recall = cum_recall/cum_total;
			
	std::vector<float> precisions_inside;	
	cum_total=0;
			std::cout << "Regions in DuDe: "<< std::endl;
			
	double fp;
	double tp_temp;
			
	for( tag2tagMapper::iterator it = DuDe_tag2mapper.begin(); it!= DuDe_tag2mapper.end(); it++ ){
		cv::Mat DuDe_segmentation_draw = cv::Mat::zeros(GT_segmentation.size(),CV_8UC1);
//				std::cout << "   " << it->first << " connected to "<< std::endl;
		tag2points inside = it->second;
		int max_intersection=0; 
		
		
		
		
		for( tag2points::iterator it2 = inside.begin(); it2!= inside.end(); it2++ ){
			
// 			for(auto it3 = it2->second.begin() ;  it3 != it2->second.end() ; ++it3){
// 				DuDe_segmentation_draw.at<uchar>(*it3) = 255;
// 			}
			
// 			total_points += it2->second.size();
			if (it2->second.size() > max_intersection) max_intersection = it2->second.size();
//					std::cout << "      " << it2->first << " with "<< it2->second.size() <<" points" << std::endl;					
		}
//				std::cout << "   max is " << max_intersection << " that represents " << 100*max_intersection/total_points   << std::endl;

		tp_temp = tp_temp + max_intersection;
		int total_points = dude_points[it->first].size();
		fp = fp + total_points - tp_temp;
		
		Precisions.push_back(100*max_intersection/total_points);
		cum_precision += max_intersection;
		cum_total += total_points;
// 		cv::imshow("mat recall", DuDe_segmentation_draw);
// 		cv::waitKey(0);
	}		
	
// 	pixel_precision=cum_precision/cum_total;
	
	std::cout << "tp_temp " << tp_temp << " tp " << tp << std::endl;
// 	assert(tp_temp == tp);
	
	
	std::vector<float> inverse_recall_inside;	
	cum_total=0;
	
	double tn_fin = 0;
	std::cout << "Regions in GT or inverse recall: "<< std::endl;
	
	for( tag2tagMapper::iterator it = DuDe_tag2mapper.begin(); it!= DuDe_tag2mapper.end(); it++ ){
		cv::Mat DuDe_segmentation_draw = cv::Mat::zeros(GT_segmentation.size(),CV_8UC1);
//				std::cout << "   " << it->first << " connected to "<< std::endl;
		tag2points inside = it->second;
		auto gt_equivalent_seg = it->second.begin();
		int max_intersection=0;
		
		for( tag2points::iterator it2 = inside.begin(); it2!= inside.end(); it2++ ){
			

// 			total_points += it2->second.size();
			if (it2->second.size() > max_intersection){
				max_intersection = it2->second.size();
				gt_equivalent_seg = it2;
			}
//					std::cout << "      " << it2->first << " with "<< it2->second.size() <<" points" << std::endl;					
		}
//				std::cout << "   max is " << max_intersection << " that represents " << 100*max_intersection/total_points   << std::endl;

		int total_points = dude_points[it->first].size();
		int total_gt_point = gt_points[gt_equivalent_seg->first].size();
		double fp = total_points - max_intersection;
		double tn = nb_of_pixels - (fp - total_gt_point);
		double inv_rec = 100* ( fp/(fp + tn) );
		
		tn_fin = tn_fin + tn;
// 				std::cout << " fp " << fp << " tn " << tn << " total_points " << total_points << " max_intersection " << max_intersection << " inverse recall " << inv_rec << std::endl;
		

		inverse_recalls.push_back( inv_rec );
		assert(fp/(fp + tn) <= 1);
		assert(fp/(fp + tn) >=0);
		cum_inverse_recall += max_intersection;
		cum_total += total_points;
		
// 				std::cout << "Up" << std::endl;
// 		cv::imshow("mat recall", DuDe_segmentation_draw);
// 		cv::waitKey(0);
	}
	
	res_out.tp = tp;
	res_out.fp = fp;
	res_out.fn = fn;
	res_out.tn = tn_fin;
	
	extract_results(res_out, Precisions, Recalls, inverse_recalls);
	
// 			std::cout << "DONE " << std::endl;
// 	pixel_inverse_recall=cum_inverse_recall/cum_total;
}


void AASS::maoris::extract_results(AASS::maoris::results& Regions, std::vector< float >& Precisions, std::vector< float >& Recalls, std::vector< float >& inverse_recall)
{
// 	pixel.precision = 100*pixel_precision;
// 	pixel.recall    = 100*pixel_recall;
	
	std::cout << "Extract res" << std::endl;
	
	float cum_precision=0;
	float cum_recall=0;
	float cum_inverse_recall=0;
	int size_precision=0, size_recall=0, size_inverse_recall = 0;

	for(int j=0; j < Precisions.size();j++){
		cum_precision += Precisions[j];
		size_precision++;
	}
	std::cout << "Done1" << std::endl;
	for(int j=0; j < Recalls.size();j++){
		cum_recall    += Recalls[j];
		size_recall++;
	}
	std::cout << "Done2" << std::endl;
	for(int j=0; j < inverse_recall.size();j++){
		cum_inverse_recall    += inverse_recall[j];
		size_inverse_recall++;
	}
	std::cout << "Done3" << std::endl;
	Regions.precision = cum_precision/Precisions.size();
	Regions.recall    = cum_recall/Recalls.size();
	Regions.inverse_recall    = cum_inverse_recall/inverse_recall.size();

	std::cout << "Done" << std::endl;

//			Precisions.clear();
//			Recalls.clear();
}

cv::Mat AASS::maoris::segment_Ground_Truth(cv::Mat GroundTruth_BW)
{
	cv::Mat src = GroundTruth_BW.clone();
	cv::Mat drawing = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);

	src = src > 250;
	
	cv::erode(src, src, cv::Mat(), cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			// erode ground truth obstacle
	
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	
	cv::findContours( src, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	
	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	int idx = 0;
	//DO NOT CHANGE THAT. WRONG THE COMPARISON LATER OTHERWISE :/
	int color= 249 / contours.size();
	int count = 1;
	std::vector<int> all_vals;
	all_vals.push_back(0);
	assert(checkAllValue(drawing, all_vals) == true);
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
// 		std::cout << "Contour" << std::endl;
		cv::drawContours( drawing, contours, idx, count , CV_FILLED, 8, hierarchy );
// 		std::cout << drawing << std::endl;
		all_vals.push_back(count);
// 		cv::imshow("Contours", drawing);
// 		cv::waitKey(0);
// 		assert(checkAllValue(drawing, all_vals) == true && "After draw con");
		count++;
// 		std::cout << "Values " << color*count << std::endl;
		
	}
	assert(checkAllValue(drawing, all_vals) == true);
	cv::dilate(drawing, drawing, cv::Mat(), cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );	
	assert(checkAllValue(drawing, all_vals) == true);		// erode dilate drawing
	return drawing;
}


bool AASS::maoris::checkAllValue(const cv::Mat& mat_in, const std::vector< int >& all_vals)
{
//	std::cout << mat_in.size().width << " " << mat_in.size().height << std::endl;
	
	for(int x=0; x < mat_in.size().width; x++){
		for(int y=0; y < mat_in.size().height; y++){
			cv::Point current_pixel(x,y);
								
			int tag_GT   = (int) mat_in.at<uchar>(current_pixel);
			bool flag = false;
			for (auto it = all_vals.begin(); it != all_vals.end() ; ++it) {
				if(*it == tag_GT){
// 					std::cout << "wel " << *it << " " << (int) tag_GT << std::endl;
					flag = true;
				}
			}
			if(flag == false){
				std::cout << tag_GT << " Doesn't exist" << std::endl << "List : ";
				for (auto it = all_vals.begin(); it != all_vals.end() ; ++it) {
					std::cout << *it << " " ;
				}
				std::cout << std::endl;
				return false;
			}
			
		}
	}
	return true;
}
