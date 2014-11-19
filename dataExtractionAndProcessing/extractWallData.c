
/* standard include files */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

/* MBIO include files */
#include "mb_status.h"
#include "mb_define.h"
#include "mb_io.h"

int main (int argc, char **argv)
{
	extern char *optarg;
	int	errflg = 0;
	int	c;
	int	help = 0;
	int	flag = 0;

        /* CSV file output variables */
	char filename[] = "../DATA/Soquel20121031/fullWallextractedData.csv";
	char* mode = "w";
	FILE * outFile;
	//outFile = fopen(filename, mode);
	//fprintf(outFile, "time, latitude, longitude, depth, heading, speed, beam_x, beam_y, beam_z\n");
	/* MBIO status variables */
	int	status = MB_SUCCESS;
	int	verbose = 0;
	int	error = MB_ERROR_NO_ERROR;
	char	*message;

	/* MBIO read control parameters */
	int	read_datalist = MB_NO;
	mb_path	read_file;
	void	*datalist;
	int	look_processed = MB_DATALIST_LOOK_UNSET;
	double	file_weight;
	int	format;
	int	pings;
	int	lonflip;
	double	bounds[4];
	int	btime_i[7];
	int	etime_i[7];
	double	btime_d;
	double	etime_d;
	double	speedmin;
	double	timegap;
	mb_path	ifile;
	mb_path	ofile;
	int	pings_get = 1;
	int	beams_bath_alloc = 0;
	int	beams_amp_alloc = 0;
	int	pixels_ss_alloc = 0;
	
	int	obeams_bath;
	int	obeams_amp;
	int	opixels_ss;
	int	oformat;

	/* MBIO read values */
	void	*imbio_ptr = NULL;
	void	*ombio_ptr = NULL;
	void	*store_ptr = NULL;
	int	kind;
	int 	derp;
	int	time_i[7];
	double	time_d;
	double	navlon;
	double	navlat;
	double	speed;
	double	heading;
	double	distance;
	double	altitude;
	double	sonardepth;
	int	beams_bath = 0;
	int	beams_amp = 0;
	int	pixels_ss = 0;
	char	*beamflag = NULL;
	double	*bath = NULL;
	double	*bathacrosstrack = NULL;
	double	*bathalongtrack = NULL;
	double	*amp = NULL;
	double	*ss = NULL;
	double	*ssacrosstrack = NULL;
	double	*ssalongtrack = NULL;
	char	comment[MB_COMMENT_MAXLINE];
	double *ttimes;
	double *angles;
	double *angles_forward;
	double *angles_null;
	double *bheave;
	double *alongtrack_offset;
	double *draft;
	double ssv;
	int	done;
	int	read_data;
         
	/* get current default values */
	status = mb_defaults(verbose,&format,&pings_get,&lonflip,bounds,
		btime_i,etime_i,&speedmin,&timegap);

        strcpy (read_file, "stdin");

	/* process argument list */
	  while ((c = getopt(argc, argv, "F:f:I:i:O:o")) != -1)
	  switch (c)
		{
		case 'F':
		case 'f':
			sscanf (optarg,"%d", &format);
			flag++;
			break;
		case 'I':
		case 'i':
			sscanf (optarg,"%s", read_file);
			flag++;
			break;
                case 'O':
                case 'o':
                        sscanf (optarg,"%s",filename);
                        flag++;
                        break;
		case '?':
			errflg++;
		}

       
	/* get format if required */
	if (format == 0)
		mb_get_format(verbose,read_file,NULL,&format,&error);

	/* determine whether to read one file or a list of files */
	if (format < 0)
		read_datalist = MB_YES;
	
	/* open file list */
	if (read_datalist == MB_YES)
	    {
	    if ((status = mb_datalist_open(verbose,&datalist,
					    read_file,look_processed,&error)) != MB_SUCCESS)
		{
		error = MB_ERROR_OPEN_FAIL;
		fprintf(stderr,"\nUnable to open data list file: %s\n",
			read_file);
		fprintf(stderr,"\nProgram Terminated\n");
		exit(error);
		}
	    if ((status = mb_datalist_read(verbose,datalist,
			    ifile,&format,&file_weight,&error))
			    == MB_SUCCESS)
		read_data = MB_YES;
	    else
		read_data = MB_NO;
	    }
	/* else copy single filename to be read */
	else
	    {
	    strcpy(ifile, read_file);
	    read_data = MB_YES;
	    }
        /* Set up output */
        outFile = fopen(filename, mode);
	fprintf(outFile, "time, latitude, longitude, depth, heading, speed, beam_x, beam_y, beam_z\n");

	/* loop over all files to be read */
	while (read_data == MB_YES)
		{
		/* initialize reading the swath file */
		if ((status = mb_read_init(
			verbose,ifile,format,pings_get,lonflip,bounds,
			btime_i,etime_i,speedmin,timegap,
			&imbio_ptr,&btime_d,&etime_d,
			&beams_bath_alloc,
			&beams_amp_alloc,
			&pixels_ss_alloc,
			&error)) != MB_SUCCESS)
			{
			mb_error(verbose,error,&message);
			fprintf(stderr,"\nMBIO Error returned from function <mb_read_init>:\n%s\n",message);
			fprintf(stderr,"\nSwath File <%s> not initialized for reading\n",ifile);
			fprintf(stderr,"\nProgram Terminated\n");
                        fprintf(stderr,
                                "\n\n\n usage: extractWallData -F-1 -I [PATH-TO-DATALIST]\n   \n");
			exit(error);
			}

		/* initialize writing the swath file */
		oformat = format;
		sprintf(ofile,"%s.mb%d",ifile,oformat);
		if ((status = mb_write_init(
			verbose,ofile,oformat,&ombio_ptr,
			&obeams_bath,&obeams_amp,&opixels_ss,&error)) != MB_SUCCESS)
			{
			mb_error(verbose,error,&message);
			fprintf(stderr,"\nMBIO Error returned from function <mb_write_init>:\n%s\n",message);
			fprintf(stderr,"\nMultibeam File <%s> not initialized for writing\n",ofile);
			fprintf(stderr,"\nProgram Terminated\n");
			exit(error);
			}
	
		/* allocate memory for data arrays */
		status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
						sizeof(char), (void **)&beamflag, &error);
		if (error == MB_ERROR_NO_ERROR)
		    status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
						sizeof(double), (void **)&bath, &error);
		if (error == MB_ERROR_NO_ERROR)
		    status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_AMPLITUDE,
						sizeof(double), (void **)&amp, &error);
		if (error == MB_ERROR_NO_ERROR)
		    status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
						sizeof(double), (void **)&bathacrosstrack, &error);
		if (error == MB_ERROR_NO_ERROR)
		    status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
						sizeof(double), (void **)&bathalongtrack, &error);
		if (error == MB_ERROR_NO_ERROR)
		    status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_SIDESCAN,
						sizeof(double), (void **)&ss, &error);
		if (error == MB_ERROR_NO_ERROR)
		    status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_SIDESCAN,
						sizeof(double), (void **)&ssalongtrack, &error);
		if (error == MB_ERROR_NO_ERROR)
		    status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_SIDESCAN,
						sizeof(double), (void **)&ssacrosstrack, &error);
		/*	Allocate memory for ttimes calls	*/
        	if (error == MB_ERROR_NO_ERROR)
                	status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
                                                sizeof(double), (void **)&ttimes, &error);
        	if (error == MB_ERROR_NO_ERROR)
                	status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
                                                sizeof(double), (void **)&angles, &error);
        	if (error == MB_ERROR_NO_ERROR)
                	status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
                                                sizeof(double), (void **)&angles_forward, &error);
        	if (error == MB_ERROR_NO_ERROR)
                	status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
                                                sizeof(double), (void **)&angles_null, &error);
        	if (error == MB_ERROR_NO_ERROR)
                	status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
                                                sizeof(double), (void **)&bheave, &error);
        	if (error == MB_ERROR_NO_ERROR)
                	status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
                                                sizeof(double), (void **)&alongtrack_offset, &error);
	        if (error == MB_ERROR_NO_ERROR)
                	status = mb_register_array(verbose, imbio_ptr, MB_MEM_TYPE_BATHYMETRY,
                                                sizeof(double), (void **)&draft, &error);
		/* read and process data */
		while (error <= MB_ERROR_NO_ERROR)
			{
			status = mb_get_all(verbose,imbio_ptr,&store_ptr,&kind,
					time_i,&time_d,&navlon,&navlat,
					&speed,&heading,
					&distance,&altitude,&sonardepth,
					&beams_bath,&beams_amp,&pixels_ss,
					beamflag,bath,amp,
					bathacrosstrack,bathalongtrack,
					ss,ssacrosstrack,ssalongtrack,
					comment,&error);
			if(kind == MB_DATA_DATA || kind == MB_DATA_NAV1 || kind == MB_DATA_NAV2){
				//fprintf(stdout,"kind:%d error:%d status:%d beams:%d\n",kind,error,status,beams_bath);
                        }
			if (kind == MB_DATA_DATA){
				status = mb_ttimes(verbose,imbio_ptr,store_ptr,&kind,&beams_bath,
					ttimes,angles,angles_forward,angles_null,
					bheave, alongtrack_offset, draft,&ssv,&error);
			}
					
			if (kind == MB_DATA_DATA && error <= MB_ERROR_NO_ERROR){
				int iq = 0;
				for (iq = 0; iq<beams_bath; iq++){
					if(ttimes[iq] > 0.0){ 
					  /*fprintf(stdout,"time: %6.2f lat: %f lon: %f psi: %f depth: %f range: %f angle:%f 						  angle forward:%f\n",time_d,navlon,navlat,heading,sonardepth,ttimes[iq]*ssv/2.,
					  angles[iq],angles_forward[iq]);*/
					  double range = ttimes[iq]*ssv/2.;
					  double xb = range*sin(M_PI/180.*angles[iq])*sin(M_PI/180.*angles_forward[iq]);
					  double yb = range*sin(M_PI/180.*angles[iq])*cos(M_PI/180.*angles_forward[iq]);
					  double zb = range*cos(M_PI/180.*angles[iq]);
					  /*fprintf(stdout,"time: %f range: %f xb: %f yb: %f zb: %f\n" , 	
						time_d,range,xb,yb,zb);*/
					/*fprintf(outFile, "time, latitude, longitude, depth, heading, speed, beam_x, beam_y, beam_z
						\n")*/
					fprintf(outFile, 
                                             "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                                             time_d,navlat,navlon,sonardepth,heading,
                                             speed,xb,yb,zb);
					}
				}
			}
			if (status == MB_SUCCESS)
			status = mb_put_all(verbose,ombio_ptr,
					store_ptr,MB_NO,kind,
					time_i,time_d,
					navlon,navlat,speed,heading,
					beams_bath,beams_amp,pixels_ss,
					beamflag,bath,amp,bathacrosstrack,bathalongtrack,
					ss,ssacrosstrack,ssalongtrack,
					comment,&error);
			}
	
		/* close the swath file */
		status = mb_close(verbose, &imbio_ptr, &error);
		status = mb_close(verbose, &ombio_ptr, &error);
			
		/* figure out whether and what to read next */
		if (read_datalist == MB_YES)
			{
			if ((status = mb_datalist_read(verbose,datalist,
				    ifile,&format,&file_weight,&error))
				    == MB_SUCCESS)
				read_data = MB_YES;
			else
				read_data = MB_NO;
			}
		else
			{
			read_data = MB_NO;
			}
	
		/* end loop over files in list */
		}
	if (read_datalist == MB_YES)
		mb_datalist_close(verbose,&datalist,&error);
      
	/* check memory */
	status = mb_memory_list(verbose,&error);

	/* Close output file */
	fclose(outFile);
 return 0;
}
