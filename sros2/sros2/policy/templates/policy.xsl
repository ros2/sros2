<?xml version="1.0" encoding="UTF-8"?>

<xsl:stylesheet version="1.0"
 xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
 xmlns:ext="http://exslt.org/common" exclude-result-prefixes="ext">
 <xsl:output omit-xml-declaration="yes" indent="yes"/>
 <xsl:strip-space elements="*"/>


<xsl:template match="node()|@*">
  <xsl:copy>
    <!-- alphabetize attributes -->
    <xsl:apply-templates select="@*">
      <xsl:sort select="name()"/>
    </xsl:apply-templates>

    <!-- alphabetize elements -->
    <xsl:apply-templates select="node()">
      <!-- by tag name -->
      <xsl:sort select="name()"/>
      <!-- by text value -->
      <xsl:sort select="text()"/>
      <!-- by namespace -->
      <xsl:sort select="concat(@ns, @node)"/>
      <!-- by path -->
      <xsl:sort select="@path"/>
    </xsl:apply-templates>
  </xsl:copy>
</xsl:template>

<!-- prune duplicate expressions -->
<xsl:template match="topic[. = preceding-sibling::topic]"/>
<xsl:template match="service[. = preceding-sibling::service]"/>
<xsl:template match="action[. = preceding-sibling::action]"/>

</xsl:stylesheet>
