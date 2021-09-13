#include <Python.h>


static PyObject* starkit_alpha_calculation(PyObject *self, PyObject *args){
		
    double xt;
    double yt;
    double zt;
	double x;
    double y;
    double z;
	double w;

	PyObject * sizes;
	PyObject * limAlpha;
	
	PyArg_ParseTuple(args, "dddddddOO", &xt, &yt, &zt, &x, &y, &z, &w, &sizes, &limAlpha);
	//PyArg_ParseTuple(args, "dO", &xt, &limAlpha);
			
	double alpha5 = w;
	
	double a6 = PyFloat_AsDouble(PyList_GetItem(sizes, 3));
	double a7 = PyFloat_AsDouble(PyList_GetItem(sizes, 4));
	double a8 = PyFloat_AsDouble(PyList_GetItem(sizes, 5));
	double a9 = PyFloat_AsDouble(PyList_GetItem(sizes, 6));
	double a5 = PyFloat_AsDouble(PyList_GetItem(sizes, 0));
	double b5 = PyFloat_AsDouble(PyList_GetItem(sizes, 1));
	double a10 = PyFloat_AsDouble(PyList_GetItem(sizes, 7));
	double b10 = PyFloat_AsDouble(PyList_GetItem(sizes, 8));
	double c10 = PyFloat_AsDouble(PyList_GetItem(sizes, 9));
	
	double cos5 = cos(alpha5);
	double sin5 = sin(alpha5);
	double nor = (sqrt)(x * x + y * y + z * z);
	
	x = x / nor;
    y = y / nor;
    z = z / nor;
	
	double xtp = xt * cos5 + (yt + a5) * sin5;
    double ytp = (yt + a5) * cos5 - xt * sin5;
    double ztp = zt;
	double xp =  x * cos5 + y * sin5;
    double yp = y * cos5 - x * sin5;
    double zp = z;
	
	// Получаем элемент из списка - он также Python-объект
    PyObject* limAlpha6 = PyList_GetItem(limAlpha, 1);
	PyObject* limAlpha10 = PyList_GetItem(limAlpha, 5);
	PyObject* limAlpha7 = PyList_GetItem(limAlpha, 2);
	PyObject* limAlpha8 = PyList_GetItem(limAlpha, 3);
	PyObject* limAlpha9 = PyList_GetItem(limAlpha, 4);

	double lim1a = PyFloat_AsDouble(PyList_GetItem(limAlpha6, 0)) * 0.00058909; //limAlpha6[0]*0.00058909
    double lim2a = PyFloat_AsDouble(PyList_GetItem(limAlpha6, 1)) * 0.00058909; //limAlpha6[1]*0.00058909
	
	int ind = 1;
	double step1 = (lim2a - lim1a) / 10;
	double step = 0;
	double alpha6 = 0;
	double alpha10 = 0;
	double temp = 0;
	double tan6 = 0;
	
	PyObject* testalpha6 = PyList_New(0);
	PyObject* angles = PyList_New(0);
	PyObject* sub_list_1 = PyList_New(0);
	PyObject* sub_list_2 = PyList_New(0);
	
	double cosinus = 0, sinus = 0;
	uint16_t i, j, ii;
	
		for (i = 0; i < 11; i++)
	{
		alpha6 = lim1a + i * step1;		
        cosinus = cos(alpha6);		
        sinus = sin(alpha6);
		temp = ((ytp + b5) * cosinus + ztp * sinus - c10) * (((yp * cosinus + zp * sinus) * (yp * cosinus + zp * sinus)) - ((zp * cosinus - yp * sinus) * (zp * cosinus - yp * sinus)) - xp * xp) - a10 \
			- (b10 * (yp * cosinus + zp * sinus) / sqrt( ((zp * cosinus - yp * sinus) * (zp * cosinus - yp * sinus)) + (xp * xp) ));
		//printf("temp = ");
		//printf("%f\n", (double)temp);		
		PyList_Append(testalpha6, Py_BuildValue("d", temp));
	}
	
	PyObject* points = PyList_New(0);
	for (i = 0; i < 10; i++)//for i in range(10):
	{		
		//if (testalpha6[i]>0 and testalpha6[i+1]<0) or (testalpha6[i]<0 and testalpha6[i+1]>0): points.append(i)
		if ((PyFloat_AsDouble(PyList_GetItem(testalpha6, i)) > 0 && PyFloat_AsDouble(PyList_GetItem(testalpha6, i + 1)) < 0) || (PyFloat_AsDouble(PyList_GetItem(testalpha6, i)) < 0 && PyFloat_AsDouble(PyList_GetItem(testalpha6, i + 1)) > 0))
		{
			PyList_Append(points, Py_BuildValue("i", i));
			//printf("i = ");
			//printf("%f\n\r", (double)i);
		}
	}
	
	uint16_t k = 0, k2 = 0;
	if (PyList_Size(points) == 0)//if len(points)==0:
	{
		for (i = 0; i < 11; i++)		//for i in range(11):
		{			
			if ( (fabs(PyFloat_AsDouble(PyList_GetItem(testalpha6, i)))) < (fabs(PyFloat_AsDouble(PyList_GetItem(testalpha6, k)))));//if (math.fabs(testalpha6[i]) < math.fabs(testalpha6[k])): k=i
			k = i;
			if (k == 10) 
				PyList_Append(points, Py_BuildValue("i", 9));//if k==10: points.append(9)				
			else
			{
				if ((fabs(PyFloat_AsDouble(PyList_GetItem(testalpha6, k - 1)))) < (fabs(PyFloat_AsDouble(PyList_GetItem(testalpha6, k + 1)))))//if (math.fabs(testalpha6[k-1]) < math.fabs(testalpha6[k+1])): points.append(k-1)
				PyList_Append(points, Py_BuildValue("i", k - 1));				
				else PyList_Append(points, Py_BuildValue("i", k));// points.append(k)
			}			
		}		
	}

	PyObject* alpha6m = PyList_New(0);//alpha6m = []
	double lim1, lim2;
	
	for (j = 0; j < PyList_Size(points); j++)//for j in range(len(points)):
	{		
		lim1 = lim1a + PyFloat_AsDouble(PyList_GetItem(points, j)) * step1;//lim1=lim1a+points[j]*step1
        lim2 = lim1 + step1;
		while(1)
		{
			step = (lim2 - lim1) / 10;
            PySequence_DelSlice(testalpha6, 0, PySequence_Length(testalpha6));//starkit_list_clear(testalpha6);    //testalpha6 = []
			for (i = 0; i < 11; i++)//for i in range (11):
			{
				alpha6 = lim1 + i * step;
				cosinus = cos(alpha6);		//cos= math.cos(alpha6)
        		sinus = sin(alpha6);       //sin= math.sin(alpha6)
				temp = ((ytp + b5) * cosinus + ztp * sinus - c10) * (pow((yp * cosinus + zp * sinus), 2) - pow((zp * cosinus - yp * sinus), 2) - xp * xp) - a10 \
				- (b10 * (yp * cosinus + zp * sinus) / sqrt( ((zp * cosinus - yp * sinus) * (zp * cosinus - yp * sinus)) + xp * xp));
				PyList_Append(testalpha6, Py_BuildValue("d", temp));//				mp_obj_list_append(testalpha6,  mp_obj_new_float(temp)); 
				//printf("temp = ");
				//printf("%f\n", (double)temp);
			}
			k = 0;
			for (i = 0; i < 11; i++)//for i in range(11):
			{				
				if ((fabs(PyFloat_AsDouble(PyList_GetItem(testalpha6, i)))) < (fabs(PyFloat_AsDouble(PyList_GetItem(testalpha6, k)))))//if (math.fabs(testalpha6[i]) < math.fabs(testalpha6[k])): k = i	
					k = i;
			}
			if (k == 0) k2 = 1;//if k==0: k2=1
				else if (k == 10) k2 = 9;//elif k==10: k2 = 9
						else
						{							
							if ((fabs(PyFloat_AsDouble(PyList_GetItem(testalpha6, k - 1)))) < (fabs(PyFloat_AsDouble(PyList_GetItem(testalpha6, k + 1)))))//if (math.fabs(testalpha6[k-1]) < math.fabs(testalpha6[k+1])): k2=k-1
								 k2 = k - 1;
							else k2 = k + 1;
						}
			alpha6 = lim1 + k * step;
			if (k > k2) 
			{
            	lim1 = lim1 + k2 * step;
                lim2 = lim1 + step;
			}
			else 
			{
			    lim1 = lim1 + k * step;
                lim2 = lim1 + step;	
			}
			if ((lim2 - lim1) < 0.00025) break;
			ind = ind + 1;			
			if (ind > (PyFloat_AsDouble(PyList_GetItem(limAlpha6, 1)) - PyFloat_AsDouble(PyList_GetItem(limAlpha6, 0)))) break; //if ind> (limAlpha6[1]- limAlpha6[0]): break		
		}
		PyList_Append(alpha6m, Py_BuildValue("d", alpha6));//alpha6m.append(alpha6)		
	}

	PyObject* alpha10m = PyList_New(0);//alpha10m =[]
	uint16_t kk = 0;
	//long len_alpha6m = PyList_Size(alpha6m);
	long len_alpha6m = PyLong_AsLong(PyLong_FromSsize_t(PyList_Size(alpha6m)));
	for (i = 0; i < len_alpha6m; i++)//for i in range (len(alpha6m)):
	{		
		tan6 = tan(PyFloat_AsDouble(PyList_GetItem(alpha6m, i - kk)));//tan6 = math.tan(alpha6m[i-kk])
		//alpha10 = math.atan((-yp-zp*tan6)/math.sqrt((zp-yp*tan6)**2+xp*xp*(1+tan6*tan6)))
		alpha10 = atan( ( - yp - zp * tan6) / ( sqrt( ((zp - yp * tan6) * (zp - yp * tan6)) + xp * xp * (1 + tan6 * tan6) ) ) );//alpha10 = math.atan((-yp-zp*tan6)/math.sqrt((zp-yp*tan6)**2+xp*xp*(1+tan6*tan6)))
		if (( (PyFloat_AsDouble(PyList_GetItem(limAlpha10, 0))) < (alpha10 * 1698)) && ((alpha10 * 1698) < PyFloat_AsDouble(PyList_GetItem(limAlpha10, 1)) ) )//if limAlpha10[0] < alpha10*1698 and alpha10*1698<limAlpha10[1]: 
		{
			PyList_Append(alpha10m, Py_BuildValue("d", alpha10));//mp_obj_list_append(alpha10m, mp_obj_new_float(alpha10));//alpha10m.append(alpha10)	
		}
		else 
		{
			PyList_SetSlice(alpha6m, i - kk, i - kk + 1, NULL);//alpha6m.pop(i-kk)			//starkit_pop(alpha6m, i - kk);
			kk = kk + 1;
		}
	}
	kk = 0;
	
	double cos6 = 0;
	double sin6 = 0;
	double alpha987 = 0;
	double sin987 = 0;
	double cos987 = 0;
	double K1 = 0;	
	double K2 = 0;	
	double m = 0;
	double temp1, temp2, temp3;
	double alpha91 = 0, alpha92 = 0;
	double alpha81 = 0, alpha82 = 0;
	double alpha71 = 0, alpha72 = 0;
	uint16_t temp71 = 0, temp72 = 0;
	uint16_t temp81 = 0, temp82 = 0;
	uint16_t temp91 = 0, temp92 = 0;	
	
	//len_alpha6m = PyList_Size(alpha6m);
	len_alpha6m = PyLong_AsLong(PyLong_FromSsize_t(PyList_Size(alpha6m)));
	//printf("temp = ");
	//printf("%f\n", (double)len_alpha6m);
	
	ii=0;
	for (ii = 0; ii < len_alpha6m; ii++)//for ii in range (len(alpha6m)):
	{		
		cos6 = cos(PyFloat_AsDouble(PyList_GetItem(alpha6m, ii - kk)));//cos6 = math.cos(alpha6m[ii-kk])
        sin6 = sin(PyFloat_AsDouble(PyList_GetItem(alpha6m, ii - kk)));//sin6 = math.sin(alpha6m[ii-kk])
        alpha987 = atan(- xp / (zp * cos6 - yp * sin6));//alpha987 = math.atan(-xp/(zp*cos6- yp*sin6))
        sin987 = sin(alpha987);//sin987 = math.sin(alpha987)
        cos987 = cos(alpha987);//cos987 = math.cos(alpha987)
        K1 = a6 * sin987 + xtp * cos987 + (ztp * cos6 - (ytp + b5) * sin6) * sin987;
		//K2 = a9+a6*cos987+(ztp*cos6-(ytp+b5)*sin6)*cos987-xtp*sin987+b10/math.cos(alpha10m[ii-kk])+((ytp+b5)*cos6+ztp*sin6-c10)*math.tan(alpha10m[ii-kk])
		K2 = a9 + a6 * cos987 + (ztp * cos6 - (ytp + b5) * sin6) * cos987 - xtp * sin987 + b10 / cos(PyFloat_AsDouble(PyList_GetItem(alpha10m, ii - kk))) + ((ytp + b5) * cos6 + ztp * sin6 - c10) * \
			tan(PyFloat_AsDouble(PyList_GetItem(alpha10m, ii - kk)));
		
		m = (K1 * K1 + K2 * K2 + a8 * a8 - a7 * a7) / (2 * a8);
		temp1 = K1 * K1 * m * m - (K1 * K1 + K2 * K2) * (m * m - K2 * K2);
		
		if (temp1 >= 0)//if temp1>=0 :
		{
			temp2 = (-K1 * m + sqrt(temp1)) / (K1 * K1 + K2 * K2);
            temp3 = (-K1 * m - sqrt(temp1)) / (K1 * K1 + K2 * K2);
			if (((fabs(temp2)) <= 1) && ((fabs(temp3) <= 1)))//if math.fabs(temp2) <= 1 and math.fabs(temp3) <= 1:
			{
				alpha91 = asin(temp2);//alpha91 = math.asin(temp2)
                alpha92 = asin(temp3);//alpha92 = math.asin(temp3)
			}
			else 
			{
				PyList_SetSlice(alpha6m, ii - kk, ii - kk + 1, NULL);//alpha6m.pop(ii-kk)			//starkit_pop(alpha6m, ii - kk);
				PyList_SetSlice(alpha10m, ii - kk, i - kk + 1, NULL); //starkit_pop(alpha10m,ii - kk);//alpha10m.pop(ii-kk)
                kk = kk + 1;
				continue;
			}
		}
		else
		{
			PyList_SetSlice(alpha6m, ii - kk, ii - kk + 1, NULL);//starkit_pop(alpha6m, ii - kk);//alpha6m.pop(ii-kk)
        	PyList_SetSlice(alpha10m, ii - kk, ii - kk + 1, NULL);//starkit_pop(alpha10m,ii - kk);//alpha10m.pop(ii-kk)//тута блин
        	kk = kk + 1;
        	continue;
		}
		//alpha81 = math.atan((K1+a8*math.sin(alpha91))/(K2+a8*math.cos(alpha91))) - alpha91
		alpha81	= atan((K1 + a8 * sin(alpha91)) / (K2 + a8 * cos(alpha91))) - alpha91;
        //alpha82 = math.atan((K1+a8*math.sin(alpha92))/(K2+a8*math.cos(alpha92))) - alpha92
		alpha82	= atan((K1 + a8 * sin(alpha92)) / (K2 + a8 * cos(alpha92))) - alpha92;
		alpha71 = alpha91 + alpha81 - alpha987;
        alpha72 = alpha92 + alpha82 - alpha987;
		//printf("alpha81 = ");
		//printf("%f\n\r", (double)alpha81);
		//printf("alpha82 = ");
		//printf("%f\n\r", (double)alpha82);
		//printf("alpha71 = ");
		//printf("%f\n\r", (double)alpha71);
		//printf("alpha72 = ");
		//printf("%f\n\r", (double)alpha72);
		
		if  ((alpha71 * 1698 < PyFloat_AsDouble(PyList_GetItem(limAlpha7, 0))) || (alpha71 * 1698 > PyFloat_AsDouble(PyList_GetItem(limAlpha7, 1))))//temp71 = alpha71*1698<limAlpha7[0] or alpha71*1698>limAlpha7[1]
		temp71 = 1; else temp71 = 0;
		
		if  ((alpha72 * 1698 < 	PyFloat_AsDouble(PyList_GetItem(limAlpha7, 0))) || (alpha72 * 1698 > PyFloat_AsDouble(PyList_GetItem(limAlpha7, 1))))//temp72 = alpha72*1698<limAlpha7[0] or alpha72*1698>limAlpha7[1]
		temp72 = 1; else temp72 = 0;
		
		if ((alpha81 * 1698 <	PyFloat_AsDouble(PyList_GetItem(limAlpha8, 0))) || (alpha81 * 1698 > PyFloat_AsDouble(PyList_GetItem(limAlpha8, 1))))//temp81 = alpha81*1698<limAlpha8[0] or alpha81*1698>limAlpha8[1]
		temp81 = 1; else temp81 = 0;
        
		if ((alpha82 * 1698 < 	PyFloat_AsDouble(PyList_GetItem(limAlpha8, 0))) || (alpha82 * 1698 > PyFloat_AsDouble(PyList_GetItem(limAlpha8, 1))))//temp82 = alpha82*1698<limAlpha8[0] or alpha82*1698>limAlpha8[1]
		temp82 = 1; else temp82 = 0;

        if ((alpha91 * 1698 < 	PyFloat_AsDouble(PyList_GetItem(limAlpha9, 0))) || (alpha91 * 1698 > PyFloat_AsDouble(PyList_GetItem(limAlpha9, 1))))//temp91 = alpha91*1698<limAlpha9[0] or alpha91*1698>limAlpha9[1]
		temp91 = 1; else temp91 = 0;

        if ((alpha92 * 1698 < 	PyFloat_AsDouble(PyList_GetItem(limAlpha9, 0))) || (alpha92 * 1698 > PyFloat_AsDouble(PyList_GetItem(limAlpha9, 1))))//temp92 = alpha92*1698<limAlpha9[0] or alpha92*1698>limAlpha9[1]
		temp92 = 1; else temp92 = 0;
		
		//printf("temp71 = ");
		//printf("%f\n\r", (double)temp71);
		//printf("temp72 = ");
		//printf("%f\n\r", (double)temp72);
		//printf("temp81 = ");
		//printf("%f\n\r", (double)temp81);
		//printf("temp82 = ");
		//printf("%f\n\r", (double)temp82);
		//printf("temp91 = ");
		//printf("%f\n\r", (double)temp91);
		//printf("temp92 = ");
		//printf("%f\n\r", (double)temp92);
		
		if ((temp71 && temp72) || (temp81 && temp82) || (temp91 && temp92) || ((temp71 || temp81 || temp91) && (temp72 || temp82 || temp92)))
		{
			PyList_SetSlice(alpha6m, ii - kk, ii - kk + 1, NULL);//starkit_pop(alpha6m, ii - kk);//alpha6m.pop(ii-kk)
            PyList_SetSlice(alpha10m, ii - kk, ii - kk + 1, NULL);//starkit_pop(alpha10m, ii - kk);//alpha10m.pop(ii-kk)
            kk = kk + 1;
			continue;
		}
		else
		{
			if (!( (temp71 || temp81 || temp91) ))
			{								
				PySequence_DelSlice(sub_list_1, 0, PySequence_Length(sub_list_1));//starkit_list_clear(sub_list_1);
				PyList_Append(sub_list_1, Py_BuildValue("d", PyFloat_AsDouble(PyList_GetItem(alpha10m, ii - kk))));//mp_obj_list_append(sub_list_1, mp_obj_new_float(get_float_from_list(alpha10m, ii - kk)));
				PyList_Append(sub_list_1, Py_BuildValue("d", alpha91));//mp_obj_list_append(sub_list_1, mp_obj_new_float(alpha91));
				PyList_Append(sub_list_1, Py_BuildValue("d", alpha81));//mp_obj_list_append(sub_list_1, mp_obj_new_float(alpha81));
				PyList_Append(sub_list_1, Py_BuildValue("d", alpha71));//mp_obj_list_append(sub_list_1, mp_obj_new_float(alpha71));
				PyList_Append(sub_list_1, Py_BuildValue("d", PyFloat_AsDouble(PyList_GetItem(alpha6m, ii - kk))));//mp_obj_list_append(sub_list_1, mp_obj_new_float(get_float_from_list(alpha6m,ii - kk)));
				PyList_Append(sub_list_1, Py_BuildValue("d", alpha5));//mp_obj_list_append(sub_list_1, mp_obj_new_float(alpha5));		
				PyList_Append(angles, sub_list_1);//mp_obj_list_append(angles, sub_list_1);				
			}			
			if (!( (temp72 || temp82 || temp92) ))
			{
				PySequence_DelSlice(sub_list_2, 0, PySequence_Length(sub_list_1));//starkit_list_clear(sub_list_2);
				PyList_Append(sub_list_2, Py_BuildValue("d", PyFloat_AsDouble(PyList_GetItem(alpha10m, ii - kk))));//mp_obj_list_append(sub_list_2, mp_obj_new_float(get_float_from_list(alpha10m, ii - kk)));
				PyList_Append(sub_list_2, Py_BuildValue("d", alpha92));//mp_obj_list_append(sub_list_2, mp_obj_new_float(alpha92));
				PyList_Append(sub_list_2, Py_BuildValue("d", alpha82));//mp_obj_list_append(sub_list_2, mp_obj_new_float(alpha82));
				PyList_Append(sub_list_2, Py_BuildValue("d", alpha72));//mp_obj_list_append(sub_list_2, mp_obj_new_float(alpha72));
				PyList_Append(sub_list_2, Py_BuildValue("d", PyFloat_AsDouble(PyList_GetItem(alpha6m, ii - kk))));//mp_obj_list_append(sub_list_2, mp_obj_new_float(get_float_from_list(alpha6m,ii - kk)));
				PyList_Append(sub_list_2, Py_BuildValue("d", alpha5));//mp_obj_list_append(sub_list_2, mp_obj_new_float(alpha5));		
				PyList_Append(angles, sub_list_2);//mp_obj_list_append(angles, sub_list_2);				
			}			
		}
	}
	return Py_BuildValue("O", angles);//return angles;	
 }

static char alpha_docs[] =
    "usage: alpha(comboSize)\n";

/* deprecated: 
static PyMethodDef uniqueCombinations_funcs[] = {
    {"uniqueCombinations", (PyCFunction)uniqueCombinations, 
     METH_NOARGS, uniqueCombinations_docs},
    {NULL}
};
use instead of the above: */

static PyMethodDef module_methods[] = {
    {"starkit_alpha_calculation", (PyCFunction) starkit_alpha_calculation, 
     METH_VARARGS, alpha_docs},
    {NULL}
};


/* deprecated : 
PyMODINIT_FUNC init_uniqueCombinations(void)
{
    Py_InitModule3("uniqueCombinations", uniqueCombinations_funcs,
                   "Extension module uniqueCombinations v. 0.01");
}
*/

static struct PyModuleDef starkit =
{
    PyModuleDef_HEAD_INIT,
    "starkit", /* name of module */
    "usage: starkit.starkit_alpha_calculation(a5, b5, c5, a6, a7, a8, a9, a10, b10, c10, sizes, limAlpha)\n", /* module documentation, may be NULL */
    -1,   /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
    module_methods
};

PyMODINIT_FUNC PyInit_starkit(void)
{
    return PyModule_Create(&starkit);
}