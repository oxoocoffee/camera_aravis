#include <glib.h>
#include <stdio.h>
#include <boost/thread.hpp>

GRegex *
arv_gv_device_get_url_regex (void)
{
static GRegex *arv_gv_device_url_regex = NULL;

	if (arv_gv_device_url_regex == NULL)
		arv_gv_device_url_regex = g_regex_new ("^(local:|file:|http:)(.+\\.[^;]+);?(?:0x)?([0-9:a-f]*)?;?(?:0x)?([0-9:a-f]*)?$",
						       G_REGEX_CASELESS, (GRegexMatchFlags)0, NULL);

	return arv_gv_device_url_regex;
}

void function()
{
  char ** tokens = NULL;
  printf("coucou\n");
  tokens = g_regex_split (arv_gv_device_get_url_regex (), "local:", (GRegexMatchFlags)0);
  if(tokens != NULL)
    printf("tokens[0] %s\n", tokens[0]);
}


int main(void)
{
//  boost::thread t(function);
  boost::thread t(boost::bind(&function));
  t.join();
 // function();
  return 0;
}
