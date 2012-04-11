<?xml version="1.0"?>

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
	<xsl:output method="html" indent="yes"/>
	<xsl:param name="code"/>
	<xsl:param name="revision"/>
	<xsl:template match="/">
		<xsl:for-each select="/root/parts/part[code=$code and revision=$revision]">
			<div id="title">
				<p>
					<table>
						<caption>
							<xsl:value-of select="name"/>
						</caption>
						<tr>
							<td rowspan="3">
								<div id="main_icon">
									<img>
										<xsl:attribute name="src">
											<xsl:value-of select="picturefile"/>
										</xsl:attribute>
									</img>
								</div>
							</td>
							<td>
								<xsl:value-of select="@xsi:type"/>
							</td>
						</tr>
						<tr>
							<td>
								<xsl:value-of select="code"/>
								<xsl:text> rev </xsl:text>
								<xsl:value-of select="revision"/>
							</td>
						</tr>
					</table>
				</p>
			</div>
			<xsl:if test="@xsi:type='article' or @xsi:type='assembly' ">
				<div id="definition_documents">
					<p>
						<xsl:for-each select="definition_documents">
							<xsl:call-template name="documents_table">
								<xsl:with-param name="title" select="'Definition Documents'"/>
							</xsl:call-template>
						</xsl:for-each>
					</p>
				</div>
			</xsl:if>
			<div id="comments">
				<p>
					<xsl:value-of select="comments"/>
				</p>
			</div>
			<xsl:if test="@xsi:type='article'">
				<div id="article_data">
					<p>
						<table>
							<caption>Article data</caption>
							<tbody>
								<tr>
									<td>Price</td>
									<td>
										<xsl:value-of select="price"/>
										<xsl:text> €</xsl:text>
									</td>
								</tr>
								<tr>
									<td>Name of supplier</td>
									<td>
										<xsl:value-of select="supplier_name"/>
									</td>
								</tr>
								<tr>
									<td>Supplier reference of the article</td>
									<td>
										<xsl:value-of select="supplier_ref"/>
									</td>
								</tr>
								<tr>
									<td>Internet link</td>
									<td>
										<a>
											<xsl:attribute name="href">
												<xsl:value-of select="hyperlink"/>
											</xsl:attribute>
											<xsl:text>...</xsl:text>
										</a>
									</td>
								</tr>
							</tbody>
						</table>
					</p>
				</div>
				<div id="supplier_documents">
					<p>
						<xsl:for-each select="supplier_documents">
							<xsl:call-template name="documents_table">
								<xsl:with-param name="title" select="'Supplier Documents'"/>
							</xsl:call-template>
						</xsl:for-each>
					</p>
				</div>
			</xsl:if>
			<xsl:if test="@xsi:type='assembly' or @xsi:type='collection' ">
				<div id="bom">
					<table>
						<caption>
							<xsl:text> Bill of Material  </xsl:text>
						</caption>
						<tbody>
						<div id="bomtree">
							<ul>
								<xsl:for-each select="is_composed_of/instance">
									<xsl:call-template name="instance_template">
										<xsl:with-param name="level" select="0"/>
										<xsl:with-param name="unfold" select="'true'"/>
									</xsl:call-template>
								</xsl:for-each>
							</ul>
						</div>
						</tbody>
					</table>
				</div>
			</xsl:if>
		</xsl:for-each>
	</xsl:template>
	<xsl:template name="documents_table">
		<xsl:param name="title"/>
		<table>
			<caption>
				<xsl:value-of select="$title"/>
			</caption>
			<thead>
				<tr>
					<th>
						<xsl:choose>
							<xsl:when test="count(document)=0">No document</xsl:when>
							<xsl:otherwise>Document</xsl:otherwise>
						</xsl:choose>
					</th>
				</tr>
			</thead>
			<tbody>
				<xsl:for-each select="document">
					<tr>
						<td>
							<a>
								<xsl:attribute name="href">
									<xsl:value-of select="filename"/>
								</xsl:attribute>
								<xsl:value-of select="name"/>
							</a>
						</td>
					</tr>
				</xsl:for-each>
			</tbody>
		</table>
	</xsl:template>
	<xsl:template name="instance_template" match="instance">
		<xsl:param name="level"/>
		<xsl:param name="unfold"/>
		<xsl:variable name="instance_code" select="code"/>
		<xsl:variable name="instance_revision" select="revision"/>
		<li>
			<a href="index.html?code={$instance_code}&amp;rev={$instance_revision}">
				<xsl:value-of select="code"/>
				<xsl:text> rev  </xsl:text>
				<xsl:value-of select="revision"/>
				<xsl:text>  -  </xsl:text>
				<xsl:value-of select="/root/parts/part[code=$instance_code and revision=$instance_revision]/name"/>
				<xsl:text>   (</xsl:text>
				<xsl:value-of select="quantity"/>
				<xsl:text>)</xsl:text>
			</a>
			<ul>
				<xsl:for-each select="/root/parts/part[code=$instance_code and revision=$instance_revision]/is_composed_of/instance">
					<xsl:call-template name="instance_template">
						<xsl:with-param name="level" select="$level+1"/>
						<xsl:with-param name="unfold" select="'true'"/>
					</xsl:call-template>
				</xsl:for-each>
			</ul>
		</li>
	</xsl:template>
</xsl:stylesheet>
