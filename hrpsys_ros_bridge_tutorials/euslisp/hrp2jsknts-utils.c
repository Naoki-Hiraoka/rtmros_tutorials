/* hrp2jsknts-utils.c :  entry=hrp2jsknts_utils */
/* compiled by EusLisp 9.23( 1.1.0) for Linux64 created on ip-172-30-1-231(Thu Feb 22 20:55:14 PST 2018) */
#include "eus.h"
#include "hrp2jsknts-utils.h"
#pragma init (register_hrp2jsknts_utils)
extern double fabs();
extern pointer fcallx();
static void init_ftab();
extern pointer loadglobal(),storeglobal();
static pointer module,*qv,codevec,quotevec;
extern pointer ___hrp2jsknts_utils();
extern pointer build_quote_vector();
static int register_hrp2jsknts_utils()
  { add_module_initializer("___hrp2jsknts-utils", ___hrp2jsknts_utils);}


/*:inverse-kinematics*/
static pointer M346hrp2jsknts_robot_inverse_kinematics(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<3) maerror();
RST348:
	ctx->vsp=local+0;
	local[0]= minilist(ctx,&argv[n],n-3);
	ctx->vsp=local+1;
	n=parsekeyparams(fqv[0], &argv[3], n-3, local+1, 1);
	if (n & (1<<0)) goto KEY349;
	ctx->vsp=local+2;
	local[2]= makeclosure(codevec,quotevec,CLO350,env,argv,local);
	local[3]= fqv[1];
	ctx->vsp=local+4;
	w=(pointer)MAPCAR(ctx,2,local+2); /*mapcar*/
	local[1] = w;
KEY349:
	local[2]= (pointer)get_sym_func(fqv[2]);
	local[3]= argv[0];
	local[4]= *(ovafptr(argv[1],fqv[3]));
	local[5]= fqv[4];
	local[6]= argv[2];
	local[7]= fqv[5];
	local[8]= local[1];
	local[9]= local[0];
	ctx->vsp=local+10;
	w=(pointer)APPLY(ctx,8,local+2); /*apply*/
	local[0]= w;
BLK347:
	ctx->vsp=local; return(local[0]);}

/*:fullbody-inverse-kinematics*/
static pointer M351hrp2jsknts_robot_fullbody_inverse_kinematics(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<3) maerror();
RST353:
	ctx->vsp=local+0;
	local[0]= minilist(ctx,&argv[n],n-3);
	ctx->vsp=local+1;
	n=parsekeyparams(fqv[6], &argv[3], n-3, local+1, 1);
	if (n & (1<<0)) goto KEY354;
	ctx->vsp=local+2;
	local[2]= makeclosure(codevec,quotevec,CLO355,env,argv,local);
	local[3]= fqv[7];
	ctx->vsp=local+4;
	w=(pointer)MAPCAR(ctx,2,local+2); /*mapcar*/
	local[1] = w;
KEY354:
	local[2]= (pointer)get_sym_func(fqv[2]);
	local[3]= argv[0];
	local[4]= *(ovafptr(argv[1],fqv[3]));
	local[5]= fqv[8];
	local[6]= argv[2];
	local[7]= fqv[5];
	local[8]= local[1];
	local[9]= local[0];
	ctx->vsp=local+10;
	w=(pointer)APPLY(ctx,8,local+2); /*apply*/
	local[0]= w;
BLK352:
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer CLO350(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= env->c.clo.env1[0];
	local[1]= argv[0];
	local[2]= fqv[9];
	local[3]= fqv[10];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,4,local+0); /*send*/
	local[0]= w;
	local[1]= makeint((eusinteger_t)0L);
	ctx->vsp=local+2;
	w=(pointer)LIST(ctx,2,local+0); /*list*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer CLO355(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= env->c.clo.env1[0];
	local[1]= argv[0];
	local[2]= fqv[9];
	local[3]= fqv[10];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,4,local+0); /*send*/
	local[0]= w;
	local[1]= makeint((eusinteger_t)0L);
	ctx->vsp=local+2;
	w=(pointer)LIST(ctx,2,local+0); /*list*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/* initializer*/
pointer ___hrp2jsknts_utils(ctx,n,argv,env)
register context *ctx; int n; pointer *argv; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv;
  register int i;
  numunion nu;
  module=argv[0];
  quotevec=build_quote_vector(ctx,QUOTE_STRINGS_SIZE, quote_strings);
  module->c.code.quotevec=quotevec;
  codevec=module->c.code.codevec;
  fqv=qv=quotevec->c.vec.v;
  init_ftab();
	local[0]= fqv[11];
	ctx->vsp=local+1;
	w=(*ftab[0])(ctx,1,local+0,&ftab[0],fqv[12]); /*load*/
	local[0]= fqv[13];
	local[1]= fqv[14];
	local[2]= fqv[15];
	ctx->vsp=local+3;
	w=(*ftab[0])(ctx,3,local+0,&ftab[0],fqv[12]); /*load*/
	local[0]= fqv[16];
	local[1]= loadglobal(fqv[17]);
	local[2]= fqv[18];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	local[1]= w;
	ctx->vsp=local+2;
	w=(*ftab[1])(ctx,2,local+0,&ftab[1],fqv[19]); /*assoc*/
	if (w!=NIL) goto IF356;
	local[0]= fqv[20];
	local[1]= loadglobal(fqv[17]);
	local[2]= fqv[18];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	local[1]= w;
	ctx->vsp=local+2;
	w=(*ftab[1])(ctx,2,local+0,&ftab[1],fqv[19]); /*assoc*/
	local[0]= w;
	local[1]= fqv[16];
	ctx->vsp=local+2;
	w=(pointer)RPLACA(ctx,2,local+0); /*rplaca*/
	local[0]= w;
	goto IF357;
IF356:
	local[0]= NIL;
IF357:
	local[0]= fqv[21];
	local[1]= fqv[17];
	ctx->vsp=local+2;
	w=(*ftab[2])(ctx,0,local+2,&ftab[2],fqv[22]); /*get-hrp2-with-hand-class-methods*/
	local[2]= w;
	local[3]= NIL;
	ctx->vsp=local+4;
	w=(pointer)APPEND(ctx,2,local+2); /*append*/
	ctx->vsp=local+2;
	w = cons(ctx,local[1],w);
	ctx->vsp=local+1;
	local[0]= cons(ctx,local[0],w);
	ctx->vsp=local+1;
	w=(pointer)EVAL(ctx,1,local+0); /*eval*/
	ctx->vsp=local+0;
	addcmethod(ctx,module,M346hrp2jsknts_robot_inverse_kinematics,fqv[4],fqv[17],fqv[23]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,M351hrp2jsknts_robot_fullbody_inverse_kinematics,fqv[8],fqv[17],fqv[24]);
	local[0]= NIL;
	ctx->vsp=local; return(local[0]);}
static void init_ftab()
{  register int i;
  for (i=0; i<3; i++) ftab[i]=fcallx;
}
