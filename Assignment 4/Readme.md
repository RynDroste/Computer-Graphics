//Author : Xuanlin Chen
//Work E-mail : chenxu@usi.ch
//Personal E-mail : kissofazshara@gmail.com
//Exercise solved: Exercies 1 , 2, 3, 4.

To deal with reflection and refraction, I add the variable reflectivity and refractiveIndex in "Material.h". 

The reflectivity is a real number that indicates the portion of the light undergoing perfect reflection. 

The refractiveIndex is defult set as 1.0, which is the refractive index of the air. 

And for the exercise only need to make one object to be refractive, I use a boolean variable to limit the specific object to be refractive.