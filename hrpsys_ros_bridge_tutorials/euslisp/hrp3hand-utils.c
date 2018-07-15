/* hrp3hand-utils.c :  entry=hrp3hand_utils */
/* compiled by EusLisp 9.23( 1.1.0) for Linux64 created on ip-172-30-1-231(Thu Feb 22 20:55:14 PST 2018) */
#include "eus.h"
#include "hrp3hand-utils.h"
#pragma init (register_hrp3hand_utils)
extern double fabs();
extern pointer fcallx();
static void init_ftab();
extern pointer loadglobal(),storeglobal();
static pointer module,*qv,codevec,quotevec;
extern pointer ___hrp3hand_utils();
extern pointer build_quote_vector();
static int register_hrp3hand_utils()
  { add_module_initializer("___hrp3hand-utils", ___hrp3hand_utils);}

static pointer F346get_hrp3hand_class_methods();
static pointer F347get_hrp2_with_hand_class_methods();

/*get-hrp3hand-class-methods*/
static pointer F346get_hrp3hand_class_methods(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	w = fqv[0];
	local[0]= w;
BLK348:
	ctx->vsp=local; return(local[0]);}

/*get-hrp2-with-hand-class-methods*/
static pointer F347get_hrp2_with_hand_class_methods(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	w = fqv[1];
	local[0]= w;
BLK349:
	ctx->vsp=local; return(local[0]);}

/* initializer*/
pointer ___hrp3hand_utils(ctx,n,argv,env)
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
	local[0]= fqv[2];
	ctx->vsp=local+1;
	w=(*ftab[0])(ctx,1,local+0,&ftab[0],fqv[3]); /*load*/
	local[0]= fqv[4];
	ctx->vsp=local+1;
	w=(*ftab[0])(ctx,1,local+0,&ftab[0],fqv[3]); /*load*/
	ctx->vsp=local+0;
	compfun(ctx,fqv[5],module,F346get_hrp3hand_class_methods,fqv[6]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[7],module,F347get_hrp2_with_hand_class_methods,fqv[8]);
	local[0]= fqv[9];
	local[1]= fqv[10];
	ctx->vsp=local+2;
	w=(pointer)F346get_hrp3hand_class_methods(ctx,0,local+2); /*get-hrp3hand-class-methods*/
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
	local[0]= fqv[9];
	local[1]= fqv[11];
	ctx->vsp=local+2;
	w=(pointer)F346get_hrp3hand_class_methods(ctx,0,local+2); /*get-hrp3hand-class-methods*/
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
	local[0]= NIL;
	ctx->vsp=local; return(local[0]);}
static void init_ftab()
{  register int i;
  for (i=0; i<1; i++) ftab[i]=fcallx;
}
