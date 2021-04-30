void main()
{
	vec3 zvec = vec3(0.0,0.0,1.0);
	vec3 posvec = normalize(vec3(gl_Vertex));
	float sinp = abs(dot(zvec,posvec));

	float rat = sqrt(1.0/(1.0+1.0*sinp));

	gl_Position = ftransform();
	gl_Position.x *= rat;
	gl_Position.y *= rat;
}
